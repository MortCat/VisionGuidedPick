"""Reattach processed D455 depth writers to saved camera prims and ROS graphs."""
import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata
import usdrt.Sdf
from isaacsim.sensors.experimental.rtx import RtxCamera, SingleViewDepthCameraSensor
from pxr import Sdf, Usd


def attach(stage):
    # Kit can retain SyntheticData graph handles from a stage closed in the
    # same process. Clear only that invalid cache before creating new sensors.
    synthetic = omni.syntheticdata.SyntheticData.Get()
    for graph in list(synthetic._nodeGraphs.values()):
        try:
            graph.get_wrapped_graph()
        except ValueError:
            synthetic.reset(usd=False)
            break
    cameras = {}
    root = stage.GetRootLayer()
    baseline = Sdf.Layer.CreateAnonymous('CameraAutoplayBaseline')
    baseline.ImportFromString(root.ExportToString())
    try:
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for arm in ('arm_01', 'arm_02'):
                body = '/World/FactoryDemo/Robots/' + arm + '/WristD455/RSD455'
                graph = '/World/FactoryDemo/ROS/' + arm + '_camera'
                if not stage.GetPrimAtPath(body) or not stage.GetPrimAtPath(graph):
                    continue
                depth_path = body + '/Camera_Pseudo_Depth'
                color_path = body + '/Camera_OmniVision_OV9782_Color'
                depth = RtxCamera(depth_path, tick_rate=15., reset_xform_op_properties=False)
                color = RtxCamera(color_path, tick_rate=15., reset_xform_op_properties=False)
                sensor = SingleViewDepthCameraSensor(depth, resolution=(480, 640),
                                                    annotators=['depth_sensor_distance'])
                cameras[arm] = {'depth': depth, 'color': color, 'sensor': sensor}
                sensor.set_enabled_post_processing(True)
                writer_name = 'FactoryAutoplay_' + arm + '_Depth'
                try:
                    rep.writers.get(writer_name)
                except Exception:
                    rep.writers.register_node_writer(
                        name=writer_name, node_type_id='isaacsim.ros2.bridge.ROS2PublishImage',
                        annotators=['DepthSensorDistance',
                                    omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                                        'IsaacReadSimulationTime',
                                        attributes_mapping={'outputs:simulationTime': 'inputs:timeStamp'})],
                        encoding='32FC1', category='factory_autoplay')
                cameras[arm]['writer'] = sensor.attach_writer(
                    writer_name, topicName='/' + arm + '/camera/depth/image_raw',
                    frameId=arm + '/camera_depth_optical_frame', nodeNamespace='', context=0, queueSize=2)
                cameras[arm]['graph'] = graph
                cameras[arm]['product'] = str(sensor.render_product.GetPath())
    except Exception:
        detach(cameras)
        raise
    finally:
        # The sensor SDK can author render infrastructure into the root layer.
        # Preserve it transiently, keeping the saved scene unchanged at runtime.
        if root.ExportToString() != baseline.ExportToString():
            transient = Sdf.Layer.CreateAnonymous('FactoryAutoplayRenderResources')
            for path in ('/Render', '/WriterOrchestrator', '/Orchestrator'):
                if root.GetPrimAtPath(path):
                    Sdf.CopySpec(root, path, transient, path)
            with Sdf.ChangeBlock():
                stage.GetSessionLayer().subLayerPaths.insert(0, transient.identifier)
                root.TransferContent(baseline)
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for camera in cameras.values():
            og.Controller.set(og.Controller.attribute(camera['graph'] + '/DepthRender.inputs:renderProductPrim'),
                              [usdrt.Sdf.Path(camera['product'])])
    return cameras


def detach(cameras):
    for camera in cameras.values():
        sensor = camera.get('sensor')
        if sensor is not None:
            sensor._invalidate_sensor()
    cameras.clear()
