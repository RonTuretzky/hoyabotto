"""Process-wide cooperative lock for the exact two physical serial devices."""
import fcntl
import hashlib
import os

class ServoOwnership:
    def __init__(self, ports):
        key=hashlib.sha256("\n".join(sorted(str(p) for p in ports)).encode()).hexdigest()[:24]
        self.path='/private/tmp/xlerobot-serial-owner-'+key+'.lock'
        self.fd=None
    def acquire(self):
        fd=os.open(self.path,os.O_CREAT|os.O_RDWR|os.O_NOFOLLOW,0o600)
        try:fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BaseException:
            os.close(fd)
            raise RuntimeError('Another canonical motor owner or read is active')
        self.fd=fd
        return self
    def close(self):
        if self.fd is not None:
            fcntl.flock(self.fd,fcntl.LOCK_UN);os.close(self.fd);self.fd=None
