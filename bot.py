#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import glob
import logging
import os
import re
import sys
from threading import Thread

from telegram import ChatAction
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, run_async

from src.config import Config
from src.downloader import Downloader

reload(sys)
sys.setdefaultencoding('utf8')

logging.basicConfig(format='[%(levelname)s] (%(name)s) %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

if len(sys.argv) < 3:
    logger.error('Please, provide all arguments!')
    exit()

config = Config(logger, sys.argv[3], sys.argv[2], sys.argv[1])
downloader = Downloader(config)


@run_async
def link(bot, update):
    text = str(update.message.text.encode('utf-8'))
    regexYoutube = \
        r"^((?:https?:)?\/\/)?((?:www|m)\.)?((?:youtube\.com|youtu.be))(\/(?:[\w\-]+\?v=|embed\/|v\/)?)([\w\-]+)(\S+)?$"
    regexSoundCloud = r"^http(s?):\/\/soundcloud.com\/(.*)\/[a-zA-Z-0-9]+$"
    patternYoutube = re.compile(regexYoutube)
    patternSoundCloud = re.compile(regexSoundCloud)
    if patternYoutube.match(text) or patternSoundCloud.match(text):
        downloader.download_by_link(bot, update, text)


def start(bot, update):
    update.message.reply_text('Send me some mp3 file and i will save it for you!')


def help(bot, update):
    update.message.reply_text('Supporting only mp3 files!')


def audio(bot, update):
    file_id = update.message.audio.file_id
    downloader.download_tg(bot, update, file_id)


def error(bot, update, e):
    logger.warning('Update "%s" caused error "%s"', update, e)

@run_async
def create_playlist(bot, update):
    message = update.message.reply_text('Start updating playlist...', quote=True)
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    for (path, subdirs, files) in os.walk(config.destination):
        os.chdir(path)
        if glob.glob("*.mp3"):
            _m3u = open(os.path.split(path)[1] + "playlist.m3u", "w")
            for song in glob.glob("*.mp3"):
                try:
                    _m3u.write(config.destination + song.encode("utf-8") + "\n")
                except:
                    logger.error('Error adding file: %s', song)
            _m3u.close()
    message.edit_text(text='Finish.')


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
    dp.add_handler(CommandHandler("playlist", create_playlist, filters=Filters.user(username=config.admin)))
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
