"""Own fresh wrist RGB and processed-depth textures for each loaded stage."""
import omni.graph.core as og
import omni.kit.app
import omni.replicator.core as rep
import omni.syntheticdata
from pxr import Sdf, Usd, UsdGeom

# Near observations use the camera clipping plane; no additional positive
# distance cutoff is imposed by the synthetic depth processing profile.
# The SDK reprojects toward negative optical X. Render from the physical
# right imager; the output then belongs to the physical left optical frame.
DEPTH_RESOLUTION = (424, 240)
DEPTH_MIN_DISTANCE = 0.0
# Baseline cadence; RGB-D synchronization must use actual render timestamps.
CAMERA_RATE_HZ = 12.0


def configure_processed_depth_frame(stage, graph, body):
    """Processed right-rendered depth is reprojected to the physical left imager."""
    parent = og.Controller.attribute(graph + '/DepthTF.inputs:parentFrameId').get()
    color_frame = og.Controller.attribute(graph + '/ColorTF.inputs:childFrameId').get()
    if parent != color_frame:
        raise RuntimeError('Depth origin correction expects a color-optical parent frame')
    cache = UsdGeom.XformCache()
    color = stage.GetPrimAtPath(body + '/Camera_OmniVision_OV9782_Color')
    left = stage.GetPrimAtPath(body + '/Camera_OmniVision_OV9782_Left')
    if not color or not left:
        raise RuntimeError('Processed-depth optical origin requires color and left imagers')
    relative = (cache.GetLocalToWorldTransform(left) *
                cache.GetLocalToWorldTransform(color).GetInverse()).ExtractTranslation()
    # USD cameras look along -Z; ROS optical frames look along +Z with Y down.
    optical_translation = (float(relative[0]), -float(relative[1]), -float(relative[2]))
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        og.Controller.attribute(graph + '/DepthTF.inputs:translation').set(optical_translation)


def initialize_rgb_writer(product, namespace, frame):
    """Publish color at render time using the same clock as processed depth."""
    writer = rep.writers.get('LdrColorSDROS2PublishImage')
    writer.initialize(topicName=namespace + '/color/image_raw', frameId=frame,
                      nodeNamespace='', context=0, queueSize=2, encoding='rgb8')
    product.factory_writer = writer


def configure_camera_rendering(stage, resources):
    """Limit this factory's sensor cadence and disable its unused textures."""
    from omni.replicator.core.scripts.utils import viewport_manager

    # Render-product creation otherwise inherits a previous Kit session's
    # MinimalRendering mode, even with a correctly configured scene file.
    import carb.settings
    settings = carb.settings.get_settings()
    settings.set('/rtx/rendermode', 'RealTimePathTracing')
    bounce_keys = ('rtx:rtpt:maxBounces', 'rtx:rtpt:cached:maxBounces',
                   'rtx:rtpt:maxSpecularAndTransmissionBounces')
    for key in bounce_keys:
        settings.set('/' + key.replace(':', '/'), 2)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for product in resources.values():
            prim = stage.GetPrimAtPath(product.path)
            prim.CreateAttribute('omni:rtx:rendermode', Sdf.ValueTypeNames.Token).Set('RealTimePathTracing')
            for key in bounce_keys:
                prim.CreateAttribute('omni:' + key, Sdf.ValueTypeNames.Int).Set(2)

    active = {product.path for product in resources.values()}
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith('/World/FactoryDemo/'):
                continue
            if prim.GetTypeName() == 'Camera' and prim.HasAttribute('omni:sensor:tickRate'):
                prim.GetAttribute('omni:sensor:tickRate').Set(CAMERA_RATE_HZ)
    # Look up existing handles only: attaching a new texture here would itself
    # create another renderer. Keep viewport products and unrelated cameras.
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.GetTypeName() != 'RenderProduct' or path in active or 'ViewportTexture' in path:
            continue
        cameras = prim.GetRelationship('camera').GetTargets()
        if not cameras or not str(cameras[0]).startswith('/World/FactoryDemo/'):
            continue
        try:
            product = viewport_manager.ViewportManager()._hydra_textures.get('', path)
        except ValueError:
            continue
        if product.hydra_texture is not None:
            product.hydra_texture.set_updates_enabled(False)


def preserve_session_resources(stage, resources):
    # The Hydra texture factory currently authors its base specs in the root,
    # even when write_to_session_layer=True. Move only our four owned specs.
    root = stage.GetRootLayer()
    transient = Sdf.Layer.CreateAnonymous('FactoryCameraRenderResources')
    paths = [Sdf.Path(product.path) for product in resources.values()]
    settings_path = '/Render/OmniverseGlobalRenderSettings'
    settings = stage.GetPrimAtPath(settings_path)
    targets = settings.GetRelationship('products').GetTargets() if settings else []
    with Sdf.ChangeBlock():
        for path in paths:
            if root.GetPrimAtPath(path):
                Sdf.CreatePrimInLayer(transient, path.GetParentPath())
                Sdf.CopySpec(root, path, transient, path)
        stage.GetSessionLayer().subLayerPaths.insert(0, transient.identifier)
        with Usd.EditContext(stage, root):
            for path in paths:
                if root.GetPrimAtPath(path):
                    stage.RemovePrim(path)
            if settings:
                settings.GetRelationship('products').SetTargets([p for p in targets if p not in paths])
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            if settings:
                settings.GetRelationship('products').SetTargets(targets)
    for product in resources.values():
        product.factory_resource_layer = transient


async def attach(stage, resources=None):
    if resources is None:
        resources = {}
    configure_camera_rendering(stage, resources)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for arm in ('arm_01', 'arm_02'):
            graph = '/World/FactoryDemo/ROS/' + arm + '_camera'
            for node in ('RGB', 'ColorInfo', 'DepthInfo'):
                og.Controller.attribute(graph + '/' + node + '.inputs:enabled').set(False)
    for _ in range(8):
        await omni.kit.app.get_app().next_update_async()
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for arm in ('arm_01', 'arm_02'):
            graph = '/World/FactoryDemo/ROS/' + arm + '_camera'
            camera = '/World/FactoryDemo/Robots/' + arm + '/WristD455/RSD455/Camera_OmniVision_OV9782_Color'
            product = rep.create.render_product(camera, (640, 480), force_new=True,
                                                name='FactoryAutoplayRGB_' + arm,
                                                write_to_session_layer=True)
            resources[arm] = product
            initialize_rgb_writer(product, '/' + arm + '/camera', arm + '/camera_color_optical_frame')
            for node in ('ColorInfo',):
                source = og.Controller.attribute(graph + '/ColorRender.outputs:renderProductPath')
                target = og.Controller.attribute(graph + '/' + node + '.inputs:renderProductPath')
                og.Controller.disconnect(source, target)
                target.set(product.path)
                og.Controller.attribute(graph + '/' + node + '.inputs:enabled').set(True)
            body = '/World/FactoryDemo/Robots/' + arm + '/WristD455/RSD455'
            depth = rep.create.render_product(body + '/Camera_OmniVision_OV9782_Right', DEPTH_RESOLUTION,
                                              force_new=True, name='FactoryAutoplayDepth_' + arm,
                                              write_to_session_layer=True)
            resources['depth_' + arm] = depth
            template = stage.GetPrimAtPath(body + '/TemplateRenderProducts/Camera_Pseudo_Depth_render_product')
            destination = stage.GetPrimAtPath(depth.path)
            for schema in template.GetAppliedSchemas():
                destination.AddAppliedSchema(schema)
            for attribute in template.GetAttributes():
                # Preserve authored sensor settings; schema fallbacks must not
                # become thousands of per-product renderer overrides.
                if (attribute.HasAuthoredValueOpinion()
                        and attribute.GetName().startswith(('omni:', 'sensor:', 'depthSensor:'))):
                    value = attribute.Get()
                    if value is not None:
                        destination.CreateAttribute(attribute.GetName(), attribute.GetTypeName()).Set(value)
            camera_prim = UsdGeom.Camera(stage.GetPrimAtPath(body + '/Camera_OmniVision_OV9782_Right'))
            focal_pixels = DEPTH_RESOLUTION[0] * camera_prim.GetFocalLengthAttr().Get() / camera_prim.GetHorizontalApertureAttr().Get()
            parameters = {'sensorSizePixel': float(DEPTH_RESOLUTION[0]),
                          'focalLengthPixel': focal_pixels, 'baselineMM': 95.,
                          'minDistance': DEPTH_MIN_DISTANCE, 'maxDistance': 2.,
                          'maxDisparityPixel': float(DEPTH_RESOLUTION[0]), 'confidenceThreshold': .05,
                          'effectiveDistance': 1., 'outlierRemovalEnabled': True}
            for key, value in parameters.items():
                destination.GetAttribute('omni:rtx:post:depthSensor:' + key).Set(value)
            writer_name = 'FactoryAutoplay_' + arm + '_Depth'
            try:
                writer = rep.writers.get(writer_name)
            except Exception:
                rep.writers.register_node_writer(
                    name=writer_name, node_type_id='isaacsim.ros2.bridge.ROS2PublishImage',
                    annotators=['DepthSensorDistance',
                                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                                    'IsaacReadSimulationTime',
                                    attributes_mapping={'outputs:simulationTime': 'inputs:timeStamp'})],
                    encoding='32FC1', category='factory_autoplay')
                writer = rep.writers.get(writer_name)
            writer.initialize(topicName='/' + arm + '/camera/depth/image_raw',
                              frameId=arm + '/camera_depth_optical_frame',
                              nodeNamespace='', context=0, queueSize=2)
            depth.factory_writer = writer
            source = og.Controller.attribute(graph + '/DepthRender.outputs:renderProductPath')
            target = og.Controller.attribute(graph + '/DepthInfo.inputs:renderProductPath')
            og.Controller.disconnect(source, target)
            target.set(depth.path)
            og.Controller.attribute(graph + '/DepthInfo.inputs:enabled').set(True)
    if stage.GetPrimAtPath('/World/FactoryDemo/Sensors/StationD455'):
        await attach_observer(stage, resources)
    preserve_session_resources(stage, resources)
    configure_camera_rendering(stage, resources)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for product in resources.values():
            writer = getattr(product, 'factory_writer', None)
            if writer is not None:
                writer.attach([product.path])
    return resources


def detach(resources):
    for product in resources.values():
        writer = getattr(product, 'factory_writer', None)
        if writer is not None:
            writer.detach()
        product.destroy()
    resources.clear()


async def attach_observer(stage, resources):
    graph='/World/FactoryDemo/ROS/station_camera';body='/World/FactoryDemo/Sensors/StationD455/RSD455'
    configure_processed_depth_frame(stage, graph, body)
    with Usd.EditContext(stage,stage.GetSessionLayer()):
        for node in ('RGB','ColorInfo','DepthInfo'):
            og.Controller.attribute(graph+'/'+node+'.inputs:enabled').set(False)
    for _ in range(8):await omni.kit.app.get_app().next_update_async()
    with Usd.EditContext(stage,stage.GetSessionLayer()):
        product=rep.create.render_product(body+'/Camera_OmniVision_OV9782_Color',(1280,720),force_new=True,name='FactoryObserverRGB',write_to_session_layer=True)
        resources['observer']=product
        initialize_rgb_writer(product, '/factory/observer', 'factory/observer_color_optical_frame')
        for node in ('ColorInfo',):
            source=og.Controller.attribute(graph+'/ColorRender.outputs:renderProductPath');target=og.Controller.attribute(graph+'/'+node+'.inputs:renderProductPath')
            og.Controller.disconnect(source,target);target.set(product.path)
            og.Controller.attribute(graph+'/'+node+'.inputs:enabled').set(True)
        depth=rep.create.render_product(body+'/Camera_OmniVision_OV9782_Right',(848,480),force_new=True,name='FactoryObserverDepth',write_to_session_layer=True)
        resources['depth_observer']=depth
        template=stage.GetPrimAtPath(body+'/TemplateRenderProducts/Camera_Pseudo_Depth_render_product');destination=stage.GetPrimAtPath(depth.path)
        for schema in template.GetAppliedSchemas():destination.AddAppliedSchema(schema)
        for attribute in template.GetAttributes():
            if attribute.HasAuthoredValueOpinion() and attribute.GetName().startswith(('omni:','sensor:','depthSensor:')) and attribute.Get() is not None:destination.CreateAttribute(attribute.GetName(),attribute.GetTypeName()).Set(attribute.Get())
        camera=UsdGeom.Camera(stage.GetPrimAtPath(body+'/Camera_OmniVision_OV9782_Right'))
        fx=848*camera.GetFocalLengthAttr().Get()/camera.GetHorizontalApertureAttr().Get()
        # The slanted gantry view sees rabbit surfaces closer than the 120 px disparity range.
        for key,value in dict(sensorSizePixel=848.,focalLengthPixel=fx,baselineMM=95.,minDistance=.24,maxDistance=2.,maxDisparityPixel=180.,confidenceThreshold=.05,effectiveDistance=1.,outlierRemovalEnabled=True).items():destination.GetAttribute('omni:rtx:post:depthSensor:'+key).Set(value)
        name='FactoryObserver_Depth'
        try:writer=rep.writers.get(name)
        except Exception:
            rep.writers.register_node_writer(name=name,node_type_id='isaacsim.ros2.bridge.ROS2PublishImage',annotators=['DepthSensorDistance',omni.syntheticdata.SyntheticData.NodeConnectionTemplate('IsaacReadSimulationTime',attributes_mapping={'outputs:simulationTime':'inputs:timeStamp'})],encoding='32FC1',category='factory_autoplay')
            writer=rep.writers.get(name)
        writer.initialize(topicName='/factory/observer/depth/image_raw',frameId='factory/observer_depth_optical_frame',nodeNamespace='',context=0,queueSize=2)
        depth.factory_writer=writer
        source=og.Controller.attribute(graph+'/DepthRender.outputs:renderProductPath');target=og.Controller.attribute(graph+'/DepthInfo.inputs:renderProductPath');og.Controller.disconnect(source,target);target.set(depth.path)
        og.Controller.attribute(graph+'/DepthInfo.inputs:enabled').set(True)
    return resources
