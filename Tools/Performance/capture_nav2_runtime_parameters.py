#!/usr/bin/env python3
"""Read node-scoped Nav2 parameters before an experiment; never set parameters."""
import argparse
import json
import time

NODES = ('/controller_server', '/planner_server', '/behavior_server', '/bt_navigator')


def assemble(records):
    controller = records['/controller_server']
    plugins = {}
    for kind in ('goal_checker_plugins', 'controller_plugins'):
        names = controller.get(kind)
        if not isinstance(names, list) or not names:
            raise ValueError('Missing runtime plugin selection: ' + kind)
        plugins[kind] = {}
        for name in names:
            values = {key[len(name) + 1:]: value for key, value in controller.items()
                      if key.startswith(name + '.')}
            if not values.get('plugin'):
                raise ValueError('Missing runtime plugin identity: ' + name)
            plugins[kind][name] = values
    return {'schema': 'crane-nav2-runtime-parameters/v1', 'source': 'ROS parameter services',
            'scope': 'pre-action readback; no parameter mutation; later changes unobserved',
            'nodes': records, 'selected_controller_plugins': plugins}


def main():
    import rclpy
    from rcl_interfaces.srv import GetParameters, ListParameters
    from rclpy.parameter import parameter_value_to_python
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=20)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('crane_runtime_parameter_capture')
    deadline = time.monotonic() + args.timeout

    def request(service, service_type, message):
        client = node.create_client(service_type, service)
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not client.wait_for_service(timeout_sec=remaining):
            raise TimeoutError(service)
        future = client.call_async(message)
        rclpy.spin_until_future_complete(node, future, timeout_sec=max(0, deadline-time.monotonic()))
        if not future.done() or future.result() is None:
            raise TimeoutError(service)
        return future.result()

    started = time.time()
    try:
        while True:
            records = {}
            for name in NODES:
                listing = ListParameters.Request()
                names = request(name + '/list_parameters', ListParameters, listing).result.names
                getting = GetParameters.Request(names=names)
                values = request(name + '/get_parameters', GetParameters, getting).values
                if len(names) != len(values):
                    raise ValueError('Parameter response length mismatch: ' + name)
                records[name] = dict(zip(names, map(parameter_value_to_python, values)))
            try:
                report = assemble(records)
                break
            except ValueError:
                # Plugin namespaces are declared by lifecycle configuration,
                # after the node's parameter services first become available.
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.1)
        report.update(started_unix=started, finished_unix=time.time())
        print(json.dumps(report, sort_keys=True, allow_nan=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
