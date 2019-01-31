#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import os
import shutil
import re
import sys
from mutagen.easyid3 import EasyID3
from threading import Thread
from slugify import slugify
import youtube_dl
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, run_async
import logging

# Enable logging
logging.basicConfig(format='%(asctime)s [%(levelname)s] (%(name)s) %(message)s',
                    level=logging.INFO)

logger = logging.getLogger(__name__)


class Config:
    def __init__(self, destination, admin, token):
        self.destination = destination + '/'
        if not os.path.exists(destination):
            logger.info('Creating destination path...')
            os.makedirs(destination)
            logger.info('Created!')
        self.admin = '@' + admin
        self.token = token

    def validate(self):
        if len(self.token) == 45 and (
                isinstance(self.admin, str) or isinstance(self.admin, unicode)) and os.path.exists(self.destination):
            return True
        return False


if len(sys.argv) < 3:
    logger.error('Please, provide all arguments!')
    exit()

config = Config(sys.argv[3], sys.argv[2], sys.argv[1])


def start(bot, update):
    update.message.reply_text('Send me some mp3 file and i will save it for you!')


def help(bot, update):
    update.message.reply_text('Supporting only mp3 files!')


def audio(bot, update):
    file_id = update.message.audio.file_id
    download_tg(bot, update, file_id)


def error(bot, update, error):
    logger.warning('Update "%s" caused error "%s"', update, error)


def youtube(bot, update):
    text = str(update.message.text.encode('utf-8'))
    regexYoutube = r"^((?:https?:)?\/\/)?((?:www|m)\.)?((?:youtube\.com|youtu.be))(\/(?:[\w\-]+\?v=|embed\/|v\/)?)([\w\-]+)(\S+)?$"
    regexSoundCloud = r"^http(s?):\/\/soundcloud.com\/(.*)\/[a-zA-Z-0-9]+$"
    patternYoutube = re.compile(regexYoutube)
    patternSoundcloud = re.compile(regexSoundCloud)
    if patternYoutube.match(text) or patternSoundcloud.match(text):
        download_youtube(bot, update, text)


def formatting(title):
    if ":" in title:
        title = title.replace(":", " -")
    if "?" in title:
        title = title.replace("?", "")
    if "/" in title:
        title = title.replace("/", "_")
    if "|" in title:
        title = title.replace("|", "_")
    if '"' in title:
        title = title.replace('"', "'")
    return slugify(title)


@run_async
def download_tg(bot, message, file_id):
    logger.info('Processing file: %s', file_id)
    newFile = bot.get_file(file_id)
    message.edit_text(text='Processing...')
    newFile.download('./temp/' + file_id)
    message.edit_text(text='Moving...')
    shutil.move('./temp/' + file_id, config.destination + '/' + file_id + '.mp3')
    message.edit_text(text='Done!')


@run_async
def download_youtube(bot, update, link):
    fileName = formatting(link)
    ydl_opts = {
        'outtmpl': config.destination + fileName + '.(ext)s',
        'format': 'bestaudio/best',
        'postprocessors': [{
            'key': 'FFmpegExtractAudio',
            'preferredcodec': 'mp3',
            'preferredquality': '320',
        }],
    }
    message = update.message.reply_text('Start downloading...', quote=True)
    with youtube_dl.YoutubeDL(ydl_opts) as ydl:
        message.edit_text(text='Downloading...')
        info = ydl.extract_info(link, download=True)
        title = info['title']
    fullTitle = title
    data = title.split(' - ', 1)
    path = config.destination + fileName + '.mp3'
    if len(data) >= 2:
        title = data[1]
        message.edit_text(text='Processing...')
        audio = EasyID3(path)
        audio["artist"] = data[0]
        audio["title"] = data[1]
        audio.save()
    if os.path.exists(path):
        fileSize = os.path.getsize(path) / (1024 * 1024) > 50
        message.edit_text(text="Sending (%sMB)" % fileSize)
        if not fileSize:
            bot.sendAudio(chat_id=update.message.chat_id, audio=open(path, 'rb'), title=title)
        else:
            message.edit_text('File is too big for sending to telegram. But i will try save in storage...')
        message.edit_text(text='Renaming...')
        shutil.move(path, config.destination + fullTitle + '.mp3')
        message.edit_text(text='Done!')
    else:
        message.edit_text(text='Error downloading.')


def main():
    updater = Updater(config.token)
    dp = updater.dispatcher

    dp.add_handler(CommandHandler("start", start))
    dp.add_handler(CommandHandler("help", help))

    def stop_and_restart():
        updater.stop()
        os.execl(sys.executable, sys.executable, *sys.argv)

    def restart(bot, update):
        update.message.reply_text('Bot is restarting...')
        Thread(target=stop_and_restart).start()

    dp.add_handler(MessageHandler(Filters.audio & Filters.user(username=config.admin), audio))
    dp.add_handler(MessageHandler(Filters.text & Filters.user(username=config.admin), youtube))

    dp.add_handler(CommandHandler('r', restart, filters=Filters.user(username=config.admin)))

    dp.add_error_handler(error)

    updater.start_polling()
    updater.idle()


if __name__ == '__main__':
    if config.validate() == True:
        main()
    else:
        logger.error('Config is not valid')
