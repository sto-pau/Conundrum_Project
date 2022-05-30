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


BASS_KEY = "drum::bass";
HIHAT_KEY = "drum::hihat";
TOM1_KEY = "drum::tom1";
TOM2_KEY = "drum::tom2";
SNARE_KEY = "drum::snare";

r = redis.Redis()
r.set(DRUM_KEY, "0")

while True:

	if (r.get(SNARE_KEY).decode('utf-8') == '1'):
		pygame.mixer.Channel(0).play(snare)
		r.set(SNARE_KEY, '0')

	if (r.get(TOM1_KEY).decode('utf-8') == '1'):
		pygame.mixer.Channel(1).play(tom1)
		r.set(TOM1_KEY, '0')

	if (r.get(TOM2_KEY).decode('utf-8') == '1'):
		pygame.mixer.Channel(2).play(tom2)
		r.set(TOM2_KEY, '0')

	if (r.get(HIHAT_KEY).decode('utf-8') == '1'):
		pygame.mixer.Channel(3).play(hihat)
		r.set(HIHAT_KEY, '0')

	if (r.get(BASS_KEY).decode('utf-8') == '1'):
		pygame.mixer.Channel(4).play(bass)
		r.set(BASS_KEY, '0')
		
