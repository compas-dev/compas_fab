import time
import threading
from compas_fab.backends import FutureResult


def test_set_result_before():
    r = FutureResult()
    r._set_result('done')
    assert(r.result() == 'done')


def test_set_result_on_thread():
    r = FutureResult()

    def set_result():
        time.sleep(0.2)
        r._set_result('done')

    t = threading.Thread(target=set_result)
    t.start()

    assert(r.result() == 'done')
