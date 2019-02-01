#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import os
import re
import sys
from threading import Thread
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, run_async
import logging
from src.config import Config
from src.downloader import Downloader

# Enable logging
logging.basicConfig(format='[%(levelname)s] (%(name)s) %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)


if len(sys.argv) < 3:
    logger.error('Please, provide all arguments!')
    exit()

config = Config(logger, sys.argv[3], sys.argv[2], sys.argv[1])

downloader = Downloader(config)


def start(bot, update):
    update.message.reply_text('Send me some mp3 file and i will save it for you!')


def help(bot, update):
    update.message.reply_text('Supporting only mp3 files!')


def audio(bot, update):
    file_id = update.message.audio.file_id
    downloader.download_tg(bot, update, file_id)


def error(bot, update, error):
    logger.warning('Update "%s" caused error "%s"', update, error)


@run_async
def link(bot, update):
    text = str(update.message.text.encode('utf-8'))
    regexYoutube = r"^((?:https?:)?\/\/)?((?:www|m)\.)?((?:youtube\.com|youtu.be))(\/(?:[\w\-]+\?v=|embed\/|v\/)?)([\w\-]+)(\S+)?$"
    regexSoundCloud = r"^http(s?):\/\/soundcloud.com\/(.*)\/[a-zA-Z-0-9]+$"
    patternYoutube = re.compile(regexYoutube)
    patternSoundcloud = re.compile(regexSoundCloud)
    if patternYoutube.match(text) or patternSoundcloud.match(text):
        downloader.download_by_link(bot, update, text)


def main():
    updater = Updater(config.token)
    dp = updater.dispatcher

    def stop_and_restart():
        updater.stop()
        os.execl(sys.executable, sys.executable, *sys.argv)

    def restart(bot, update):
        update.message.reply_text('Bot is restarting...')
        Thread(target=stop_and_restart).start()

    dp.add_handler(CommandHandler("start", start))
    dp.add_handler(CommandHandler("help", help))
    dp.add_handler(MessageHandler(Filters.audio & Filters.user(username=config.admin), audio))
    dp.add_handler(MessageHandler(Filters.text & Filters.user(username=config.admin), link))
    dp.add_handler(CommandHandler('r', restart, filters=Filters.user(username=config.admin)))
    dp.add_error_handler(error)

    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    if config.validate():
        main()
    else:
        logger.error('Config is not valid')
