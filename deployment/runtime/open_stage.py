"""Kit --exec entry: open the explicit stage, never start the timeline."""
import asyncio
import hashlib
import json
import os
from pathlib import Path
import time
from fractions import Fraction
import omni.kit.app
import omni.graph.core as og
import omni.usd
import carb.settings
from pxr import Sdf


async def open_stage():
    app = omni.kit.app.get_app()
    deadline = time.monotonic() + 180.
    # Loading a graph during Kit startup can leave its playback nodes unevaluated.
    # Wait for the full application, not just the image node registration.
    while (not app.is_app_ready()
           or not app.get_extension_manager().is_extension_enabled('isaacsim.ros2.bridge')
           or og.get_node_type('isaacsim.ros2.bridge.ROS2PublishImage') is None):
        if time.monotonic() >= deadline:
            raise RuntimeError('ros_bridge_unavailable: Kit/ROS Bridge startup was not ready; scene was not opened')
        await app.next_update_async()
    path = os.environ['PTC_RUNTIME_SCENE']
    factory_scene_dir = Path(__file__).resolve().parents[1] / 'factory_rabbit_20261005/scene'
    trusted_factory_scene = (factory_scene_dir / 'FactoryRabbitDemo.camera-bracket-v2.usda').resolve()
    is_factory = Path(path).resolve() == trusted_factory_scene
    if is_factory:
        # Fabric-backed graph histories read their period when the stage opens.
        # Changing only the timeline target later leaves their original 60 Hz dt.
        rate = Sdf.Layer.FindOrOpen(path).timeCodesPerSecond
        period = Fraction(1.0 / rate).limit_denominator(1_000_000_000)
        settings = carb.settings.get_settings()
        settings.set('/app/settings/fabricDefaultSimPeriodNumerator', period.numerator)
        settings.set('/app/settings/fabricDefaultSimPeriodDenominator', period.denominator)
        if os.name == 'nt':
            # Keep Windows rendering asynchronous during Play, rather than
            # letting the editor's throttling extension toggle it at Play.
            settings.set('/exts/isaacsim.core.throttling/enable_async', False)
            for key in ('/app/asyncRendering', '/app/asyncRenderingLowLatency',
                        '/app/omni.usd/asyncHandshake', '/omni/replicator/asyncRendering'):
                settings.set(key, True)
    opened = await omni.usd.get_context().open_stage_async(path)
    if not (opened[0] if isinstance(opened, tuple) else opened):
        raise RuntimeError('Stage open failed: ' + path)
    # This launcher already executes the trusted repository's runtime code.
    # Allow its embedded ScriptNode for this session, only for our exact scene.
    if is_factory:
        from isaacsim.core.rendering_manager import RenderingManager
        RenderingManager.set_dt(1.0 / rate)
        carb.settings.get_settings().set('/app/omni.graph.scriptnode/opt_in', True)
    scene = Path(path)
    sources = [scene, scene.parent / 'factory_base.usda',
               *(scene.parent / 'runtime' / name for name in
                 ('feeder_node.py', 'factory_runtime.py', 'rgb_runtime.py', 'bin_load_runtime.py'))]
    print('PTC_RUNTIME_SCENE ' + json.dumps({'loaded_root_layer': omni.usd.get_context().get_stage().GetRootLayer().identifier,
          'runtime_profile_revision': os.environ['PTC_RUNTIME_PROFILE_REVISION'],
          'source_sha256': {str(source): hashlib.sha256(source.read_bytes()).hexdigest() for source in sources if source.is_file()}}))


asyncio.ensure_future(open_stage())
