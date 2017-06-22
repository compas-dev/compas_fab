import yaml
import json
import logging
from timeit import default_timer as timer
from multiprocessing import Process, Manager, freeze_support
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from compas_fabrication.fabrication.robots.rfl import SimulationCoordinator, SimulationError

logging.basicConfig(format='%(process)d [%(levelname)s] %(name)s: %(message)s', level=logging.DEBUG)
LOG = logging.getLogger()


def execute_path_plan(path_list, options, host, port):
    LOG.info('Starting path plan instance on %s:%d' % (host, port))
    try:
        path = SimulationCoordinator.local_executor(options, host=host, port=port)
        for path_step in path:
            path_list.extend(path_step.raw)
    except SimulationError as e:
        LOG.exception('Path planning failed: %s', e.message)


def build_handler(instances):
    if not instances:
        instances = '127.0.0.1:19997'
    instances = [tuple(i.split(':')) for i in instances.split(',')]

    ESCALATION_STRATEGY = [{'shallow_state_search': True},
                           {'shallow_state_search': False, 'algorithm': 'rrtconnect', 'trials': 20},
                           {'shallow_state_search': False, 'algorithm': 'sbl', 'trials': 20},
                           {'shallow_state_search': False, 'algorithm': 'sbl', 'trials': 3},
                           ]

    class PathPlanningHandler(BaseHTTPRequestHandler):
        def _set_headers(self, code=200):
            self.send_response(code)
            self.send_header('Content-type', 'application/json')
            self.end_headers()

        def do_POST(self):
            LOG.info('Request on %s', self.path)
            if self.path.startswith('/path-planner'):
                start_time = timer()
                content_length = int(self.headers.getheader('content-length'))
                post_data = self.rfile.read(content_length)
                # We use yaml safe loader because unicode and python
                options = yaml.safe_load(post_data)
                LOG.debug('Received %d bytes of data' % content_length)

                manager = Manager()
                processes = []
                results = []

                for i, (host, port) in enumerate(instances):
                    port = int(port)
                    escalation_level = i % len(ESCALATION_STRATEGY)
                    escalation_options = dict(options.items() + ESCALATION_STRATEGY[escalation_level].items())

                    raw_path = manager.list()
                    process = Process(target=execute_path_plan, args=(raw_path, escalation_options, host, port,))
                    process.daemon = True
                    process.start()
                    processes.append(process)
                    results.append((raw_path, host, port))

                for process in processes:
                    process.join()

                end_time = timer()

                # Filter empty paths
                paths = sorted(filter(lambda p: len(p[0]), results), key=lambda p: len(p[0]))
                LOG.info('Found %d paths in %.2f sec' % (len(paths), (end_time - start_time)))

                self._set_headers()
                json.dump([list(path) for path, _host, _port in paths], self.wfile)

            else:
                self.send_error(404, 'File Not Found: %s' % self.path)

    return PathPlanningHandler


if __name__ == '__main__':
    from sys import argv

    if len(argv) < 1:
        print("Usage: python web_api.py [http_port=7000] [instances=127.0.0.1:19997]")
        print("   eg: python web_api.py 7000 127.0.0.1:19997,127.0.0.1:19998")
        exit(-1)

    http_port = int(argv[1]) if len(argv) > 1 else 7000
    instances = argv[2] if len(argv) > 2 else None

    httpd = HTTPServer(('0.0.0.0', http_port), build_handler(instances))
    LOG.info('Starting http server on port %d, use <Ctrl-C> to stop...', http_port)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass

    LOG.info('Exiting...')
