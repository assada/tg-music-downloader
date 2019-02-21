from __future__ import unicode_literals
from mpd import MPDClient

import os


class Config:
    def __init__(self, logger, token, admin, destination, persist, quality, mpd_host=False, mpd_port=False):
        self.destination = destination
        if not self.destination.endswith('/'):
            self.destination = destination + '/'
        if ',' in admin:
            admin = admin.split(',')
        self.admin = admin
        self.token = token
        self._logger = logger
        self.persist = persist
        self.quality = quality
        self.client = None
        if mpd_host is not False and mpd_port is not False:
            client = MPDClient()
            client.timeout = 20
            client.idletimeout = None
            client.connect(mpd_host, mpd_port)
            self.client = client
        self._create_path(self.destination)

    def _create_path(self, destination):
            if not os.path.exists(destination):
                try:
                    self.get_logger().info('Creating destination path: %s...', destination)
                    os.makedirs(destination)
                    self.get_logger().info('Created!')
                except:
                    self.get_logger().error('Error creating destination path')

    def validate(self):
        if (self.token is not None and self.admin is not None and self.destination is not None) \
                and len(self.token) == 45 \
                and (isinstance(self.admin, str) or isinstance(self.admin, unicode) or isinstance(self.admin, list)) \
                and os.path.exists(self.destination):
            return True
        return False

    def get_logger(self):
        return self._logger
