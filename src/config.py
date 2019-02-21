from __future__ import unicode_literals
from mpd import MPDClient, ConnectionError

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
        self._client = None
        self.mpd_host = mpd_host
        self.mpd_port = mpd_port
        self._connect_retries = 0
        self._create_path(self.destination)
        self.client()

    def client(self):
        if self._client is None and self.mpd_host is not False and self.mpd_port is not False:
            client = MPDClient()
            client.timeout = 20
            client.idletimeout = None
            client.connect(self.mpd_host, self.mpd_port)
            self._client = client
            self._connect_retries = 0
        if self._client is not None:
            try:
                self._client.ping()
            except ConnectionError:
                self._client = None
                if self._connect_retries <= 3:
                    self.get_logger().warning('Reconnecting to MPD...')
                    return self.client()

        return self._client

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
