#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import unicode_literals

import glob
import logging
import os
import re
import sys
from threading import Thread

import telegram
from telegram import ChatAction
from telegram.ext import Updater, CommandHandler, MessageHandler, Filters, run_async, CallbackQueryHandler

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
    os.getenv('BOT_PERSISTENCE', Helper.list_get(sys.argv, 6, True)),
    os.getenv('BOT_QUALITY', Helper.list_get(sys.argv, 7, '320')),
    mpd_host=os.getenv('BOT_MPD_HOST', Helper.list_get(sys.argv, 4, False)),
    mpd_port=os.getenv('BOT_MPD_PORT', Helper.list_get(sys.argv, 5, False))
)
downloader = Downloader(config)

updates = {}

button_list = [
            telegram.InlineKeyboardButton("▶ Play", callback_data='mpd_play'),
            telegram.InlineKeyboardButton("⏸ Pause", callback_data='mpd_pause'),
            telegram.InlineKeyboardButton("🗃 All Playlists", callback_data='mpd_lists'),
            telegram.InlineKeyboardButton("🔊 Up +5", callback_data='mpd_up'),
            telegram.InlineKeyboardButton("🔇 Mute", callback_data='mpd_mute'),
            telegram.InlineKeyboardButton("🔉 Down -5", callback_data='mpd_down')
        ]
reply_markup = telegram.InlineKeyboardMarkup(Helper.build_menu(button_list, n_cols=3))


@run_async
def link(bot, update):
    text = str(update.message.text.encode('utf-8'))
    if _validate_link(text):
        updates.update({text: update})
        downloader.question(bot, update, text)


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


def button(bot, update):
    query = update.callback_query
    data = query.data
    if data is not "cancel":
        if data.startswith('http'):
            bot.answerCallbackQuery(query.id, text='⏳')
            downloader.download_by_link(bot, updates[data], data)
        elif 'mpd_mute' in data:
            if config.client() is not None:
                config.client().setvol(0)
                bot.answerCallbackQuery(query.id, text='🔇 Mute')
        elif 'mpd_up' in data:
            if config.client() is not None:
                c_vol = config.client().status()['volume']
                if int(c_vol) <= 100:
                    config.client().setvol(int(c_vol)+5)
                    bot.answerCallbackQuery(query.id, text='🔊 +5')
        elif 'mpd_down' in data:
            if config.client() is not None:
                c_vol = config.client().status()['volume']
                if int(c_vol) != 0:
                    config.client().setvol(int(c_vol)-5)
                    bot.answerCallbackQuery(query.id, text='🔉 -5')
        elif 'mpd_play' in data:
            if config.client() is not None:
                config.client().play()
                bot.answerCallbackQuery(query.id, text='▶ Play')
        elif 'mpd_pause' in data:
            if config.client() is not None:
                config.client().pause()
                bot.answerCallbackQuery(query.id, text='⏸ Pause')
        elif 'mpd_lists' in data:
            if config.client() is not None:
                playlists(bot, query)
                bot.answerCallbackQuery(query.id, text='🥳')
        elif 'mpd_list:' in data:
            playlist = data.split(':', 1)[1]
            config.client().clear()
            config.client().load(playlist)
            config.client().play()
            bot.answerCallbackQuery(query.id, text='Playing %s' % playlist)

        if 'mpd_up' in data or 'mpd_down' in data or 'mpd_mute' in data or 'mpd_play' in data or 'mpd_pause' in data:
            if config.client() is not None:
                stat = config.client().status()
                song = config.client().currentsong()
                if 'title' in song:
                    song = song['title']
                else:
                    song = '-'
                query.message.edit_text(
                    text='*Status:* %s\n*Volume:* %s\n*Song:* %s' % (stat['state'].capitalize(), stat['volume'], song),
                    parse_mode='Markdown',
                    reply_markup=reply_markup
                )
    else:
        query.message.delete()
    if data in updates:
        del updates[data]


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
                    _m3u.write(os.path.abspath(song.encode("utf-8")) + "\n")
                except:
                    logger.error('Error adding file: %s', song)
            _m3u.close()
    message.edit_text(text='Finish.')


@run_async
def playlists(bot, update):
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    if config.client() is not None:
        plays = config.client().listplaylists()
        buttons = []
        for playlist in plays:
            buttons.append(
                telegram.InlineKeyboardButton('🎵 %s' % playlist['playlist'], callback_data='mpd_list:%s' % playlist['playlist'])
            ),
        update.message.reply_text(
            text='Choose playlist:',
            reply_markup=telegram.InlineKeyboardMarkup(Helper.build_menu(buttons, n_cols=2))
        )


@run_async
def control(bot, update):
    bot.sendChatAction(chat_id=update.message.chat_id, action=ChatAction.TYPING)
    if config.client() is not None:
        stat = config.client().status()
        song = config.client().currentsong()
        if 'title' in song:
            song = song['title']
        else:
            song = '-'
        update.message.reply_text(
            text='*Status:* %s\n*Volume:* %s\n*Song:* %s' % (stat['state'].capitalize(), stat['volume'], song),
            parse_mode='Markdown',
            reply_markup=reply_markup
        )


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

    dp.add_handler(CommandHandler("control", control, filters=Filters.user(username=config.admin)))
    dp.add_handler(CommandHandler("start", start, filters=Filters.user(username=config.admin)))
    dp.add_handler(CommandHandler("help", help, filters=Filters.user(username=config.admin)))
    dp.add_handler(CallbackQueryHandler(button))
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
