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

from src.helper import Helper
from src.config import Config
from src.downloader import Downloader

reload(sys)
sys.setdefaultencoding('utf8')

logging.basicConfig(format='[%(levelname)s] (%(name)s) %(message)s', level=os.getenv('LOG_LEVEL', logging.INFO))
logger = logging.getLogger(__name__)

config = Config(
    logger,
    os.getenv('BOT_TOKEN', Helper.list_get(sys.argv, 1, None)),
    os.getenv('BOT_ADMIN', Helper.list_get(sys.argv, 2, None)),
    os.getenv('BOT_DESTINATION', Helper.list_get(sys.argv, 3, None)),
    os.getenv('BOT_PERSISTENCE', True),
)
downloader = Downloader(config)


@run_async
def link(bot, update):
    text = str(update.message.text.encode('utf-8'))
    if _validate_link(text):
        downloader.download_by_link(bot, update, text)


def _validate_link(text):
    regex_list = [
        r"^((?:https?:)?\/\/)?((?:www|m)\.)?((?:youtube\.com|youtu.be))(\/(?:[\w\-]+\?v=|embed\/|v\/)?)([\w\-]+)(\S+)?$",
        r"^http(s?):\/\/soundcloud.com\/(.*)\/[a-zA-Z-0-9]+$"
    ]
    for reg in regex_list:
        pattern = re.compile(reg)
        if pattern.match(text):
            return True

    return False


def start(bot, update):
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    update.message.reply_text('Send me some mp3 file, YouTube or SoundCloud link and i will save it for you!')


def help(bot, update):
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    update.message.reply_text('Supporting only mp3 files, YouTube and SoundCloud links!')


def audio(bot, update):
    file_id = update.message.audio.file_id
    downloader.download_tg(bot, update, file_id)


def error(bot, update, e):
    logger.warning('Update "%s" caused error "%s"', update, e)


@run_async
def create_playlist(bot, update):
    message = update.message.reply_text('Start updating playlist...', quote=True)
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    for (path, sub_dirs, files) in os.walk(config.destination):
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
        bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
        Thread(target=stop_and_restart).start()

    dp.add_handler(CommandHandler("start", start))
    dp.add_handler(CommandHandler("help", help))
    dp.add_handler(CommandHandler("playlist", create_playlist, filters=Filters.user(username=config.admin)))
    dp.add_handler(CommandHandler('r', restart, filters=Filters.user(username=config.admin)))
    dp.add_handler(MessageHandler(Filters.text & Filters.user(username=config.admin), link))
    if config.persist:
        dp.add_handler(MessageHandler(Filters.audio & Filters.user(username=config.admin), audio))
    dp.add_error_handler(error)
    logger.info('Adding handlers done.',)
    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    if config.validate():
        logger.info(
            'Starting telegram bot (token=%s, admin=%s, destination=%s, persist=%s)',
            config.token, config.admin, config.destination, config.persist
        )
        main()
    else:
        logger.error('Config is not valid')
