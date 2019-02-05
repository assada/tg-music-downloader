from __future__ import unicode_literals

import os


class Config:
    def __init__(self, logger, token, admin, destination, persist, quality):
        self.destination = destination
        if not self.destination.endswith('/'):
            self.destination = destination + '/'
        self.admin = '@' + admin
        self.token = token
        self._logger = logger
        self.persist = persist
        self.quality = quality
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
                and (isinstance(self.admin, str) or isinstance(self.admin, unicode)) \
                and os.path.exists(self.destination):
            return True
        return False

    def get_logger(self):
        return self._logger
