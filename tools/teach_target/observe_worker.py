"""Owned Observe worker. stdout is JSON protocol; model/ROS logging goes to stderr."""
import contextlib
import importlib.util
import json
from pathlib import Path
import sys


def main():
    protocol = sys.stdout
    def emit(value):
        protocol.write(json.dumps(value, allow_nan=False) + '\n')
        protocol.flush()
    with contextlib.redirect_stdout(sys.stderr):
        model = None
        descriptor = None
        activation = None
        for line in sys.stdin:
            try:
                command = json.loads(line)
                if command['command'] == 'load_model':
                    descriptor = command['model']
                    project = Path(command['project'])
                    source = project / 'ros2_ws/src/ptc_inference/ptc_inference/observe_model.py'
                    spec = importlib.util.spec_from_file_location('observe_model', source)
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    model = module.ObserveModelAdapter(descriptor, command['config']['device'])
                    # Smoke inference on a generated frame proves the loader and mask
                    # postprocessor execute without training images or job records.
                    # No-detection is a valid result, not a fake quality acknowledgement.
                    from PIL import Image
                    side = descriptor['preprocessing']['resolution']
                    model.predict(Image.new('RGB', (side, side), (114, 114, 114)),
                                  command['config']['confidence_threshold'])
                    activation = command['activation_id']
                    emit(dict(event='loaded', activation_id=activation, model_version=descriptor['model_version'],
                              checkpoint_sha256=descriptor['checkpoint_sha256'], inference_contract=model.contract))
                elif command['command'] == 'start_run':
                    if model is None or command['activation_id'] != activation:
                        raise ValueError('No matching resident model')
                    # The stream adapter is deliberately separate from legacy EvaluateView.
                    source = project / 'ros2_ws/src/ptc_inference/ptc_inference/observe_stream.py'
                    if not source.is_file():
                        raise RuntimeError('Gantry stream adapter is not installed')
                    spec = importlib.util.spec_from_file_location('observe_stream', source)
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)
                    module.run(model, descriptor, command['run_id'], config, emit)
                    return
                elif command['command'] == 'stop_run':
                    return
                else:
                    raise ValueError('Unknown owned-worker command')
                config = command.get('config', config if 'config' in locals() else {})
            except Exception as error:
                import traceback
                traceback.print_exc(file=sys.stderr)
                emit(dict(event='error', message=str(error)))
                return


if __name__ == '__main__':
    main()
