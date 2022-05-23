import os
import random
import sys
import redis
import time

from pyglet.gl import *
import pyglet
from pyglet.window import key
 
 
window = pyglet.window.Window()
 
BASS_SOUND = 'sound/Bass.mp3'
SNARE_SOUND = 'sound/Snare.mp3'
HIHAT_SOUND = 'sound/Hihat.mp3'
TOM1_SOUND = 'sound/Tom1.mp3'
TOM2_SOUND = 'sound/Tom2.mp3'

DRUM_KEY = 'drum::key'

bass = pyglet.resource.media(BASS_SOUND, streaming=False)
snare = pyglet.resource.media(SNARE_SOUND, streaming=False)
hihat = pyglet.resource.media(HIHAT_SOUND, streaming=False)
tom1 = pyglet.resource.media(TOM1_SOUND, streaming=False)
tom2 = pyglet.resource.media(TOM2_SOUND, streaming=False)

r = redis.Redis()
r.set(DRUM_KEY, "0")

while False:
	val = r.get(DRUM_KEY).decode('utf-8')
	#print(val)
	#if (r.get(DRUM_KEY).decode('utf-8') == '1'):
	#	snare.play()
	#	r.set(DRUM_KEY, '0')
	#if (r.get(DRUM_KEY).decode('utf-8') == '2'):
	#	tom1.play()
	#	r.set(DRUM_KEY, '0')
	#if (r.get(DRUM_KEY).decode('utf-8') == '3'):
	#	tom2.play()
	#	r.set(DRUM_KEY, '0')
	#if (r.get(DRUM_KEY).decode('utf-8') == '4'):
	#	hihat.play()
	#	r.set(DRUM_KEY, '0')
		
	if (val == '1'):
		snare.play()
		r.set(DRUM_KEY, '0')
	if (val == '2'):
		tom1.play()
		r.set(DRUM_KEY, '0')
	if (val == '3'):
		tom2.play()
		r.set(DRUM_KEY, '0')
	if (val == '4'):
		hihat.play()
		r.set(DRUM_KEY, '0')
		
	

@window.event
def on_key_press(symbol, modifiers):
    if symbol == key.A:
        print("A Key Was Pressed")
        bass.play()
 
    elif symbol == key.S:
        print("S Key Was Pressed")
        snare.play()
 
    elif symbol == key.D:
        print("D Key Was Pressed")
        hihat.play()
        time.sleep(0.02)
        snare.play()
        #tom1.play()
 
 
 
@window.event
def on_draw():
    window.clear()
 
 
pyglet.app.run()
