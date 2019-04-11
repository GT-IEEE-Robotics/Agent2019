import threading

class StarveSafeReadWriteLock:
    """ A lock object that allows many simultaneous "read locks", but
    only one "write lock." """

    def __init__(self):
        self._read_ready = threading.Condition(threading.Lock())
        self._readers = 0
        self._writer_waiting = 0

    def acquire(self):
        """ Acquire a read lock. Blocks only if a thread has
        acquired the write lock. """
        self._read_ready.acquire()

        while self._writer_waiting == 1:
            self._read_ready.wait()

        try:
            self._readers += 1
        finally:
            self._read_ready.release()

    def release(self):
        """ Release a read lock. """
        self._read_ready.acquire()
        try:
            self._readers -= 1
            if not self._readers:
                self._read_ready.notifyAll()
        finally:
            self._read_ready.release()

    def acquire_write(self):
        """ Acquire a write lock. Blocks until there are no
        acquired read or write locks. """
        self._read_ready.acquire()
        self._writer_waiting = 1
        
        while self._readers > 0:
            self._read_ready.wait()

    def release_write(self):
        """ Release a write lock. """
        try:
            self._writer_waiting = 0
            self._read_ready.notifyAll()
        finally:
            self._read_ready.release()