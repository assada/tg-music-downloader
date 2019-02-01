from __future__ import unicode_literals

import os


class Config:
    def __init__(self, logger, destination, admin, token):
        self.destination = destination
        if not self.destination.endswith('/'):
            self.destination = destination + '/'
        if not os.path.exists(destination):
            logger.info('Creating destination path...')
            os.makedirs(destination)
            logger.info('Created!')
        self.admin = '@' + admin
        self.token = token
        self.logger = logger

    def validate(self):
        if len(self.token) == 45 and (
                isinstance(self.admin, str) or isinstance(self.admin, unicode)) and os.path.exists(self.destination):
            return True
        return False

    def get_logger(self):
        return self.logger
