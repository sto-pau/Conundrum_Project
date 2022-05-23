import os
import random
import sys
import redis
import time
import pygame

pygame.mixer.pre_init(44100, -16, 1, 512)
pygame.mixer.init()
pygame.init()
 
BASS_SOUND = 'sound/Bass.ogg'
SNARE_SOUND = 'sound/Snare.ogg'
HIHAT_SOUND = 'sound/Hihat.ogg'
TOM1_SOUND = 'sound/Tom1.ogg'
TOM2_SOUND = 'sound/Tom2.ogg'

bass = pygame.mixer.Sound(BASS_SOUND)
snare = pygame.mixer.Sound(SNARE_SOUND)
hihat = pygame.mixer.Sound(HIHAT_SOUND)
tom1 = pygame.mixer.Sound(TOM1_SOUND)
tom2 = pygame.mixer.Sound(TOM2_SOUND)

DRUM_KEY = 'drum::key'



r = redis.Redis()
r.set(DRUM_KEY, "0")

while True:
	val = r.get(DRUM_KEY).decode('utf-8')	
	if (val == '1'):
		pygame.mixer.Channel(0).play(snare)
		r.set(DRUM_KEY, '0')
	if (val == '2'):
		pygame.mixer.Channel(1).play(tom1)
		r.set(DRUM_KEY, '0')
	if (val == '3'):
		pygame.mixer.Channel(2).play(tom2)
		r.set(DRUM_KEY, '0')
	if (val == '4'):
		pygame.mixer.Channel(3).play(hihat)
		r.set(DRUM_KEY, '0')
	if (val == '5'):
		pygame.mixer.Channel(3).play(bass)
		r.set(DRUM_KEY, '0')
		
