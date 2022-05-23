import threading as th
import redis

from playsound import playsound
print('Here')

# def snare():
#     r = redis.Redis(host='localhost', port=6379)
#     r.set('snare_key', '0')
#     while True:
#         val = r.get('drum_key')
#         if (val == '1' or val == '2'):
#             playsound('C:\E_Drive\Classes\CS 225\Project\Snare.mp3')
#             r.set('drum_key', '0')
#             # r.set('accept_sound_key', '1')

# def bass():
#     r = redis.Redis(host='localhost', port=6379)
#     while True:
#         if (r.get('bass_key') == '1'):
#             playsound('C:\E_Drive\Classes\CS 225\Project\Bass.mp3')
#             r.set('bass_key', '0')              

# def tom2():
#     r = redis.Redis(host='localhost', port=6379)
#     while True:
#         if (r.get('tom2_key') == '1'):
#             playsound('C:\E_Drive\Classes\CS 225\Project\Tom2.mp3')
#             r.set('tom2_key', '0') 

# if __name__ == "__main__":
# # playsound('C:\E_Drive\Classes\CS 225\Project\Bass.mp3',False)

#     snareThread = th.Thread( target = snare, args = ())
#     bassThread = th.Thread( target = bass, args = ())
#     tom2Thread = th.Thread( target = tom2, args = ())

#     snareThread.start()
#     bassThread.start()
#     tom2Thread.start()
#     snareThread.join()
#     bassThread.join()
#     tom2Thread.join()



r = redis.Redis(host='localhost', port=6379)
r.set('drum_key', "0")
while False:
	val = r.get('drum_key')
	print("Current value is ", val)
	if(val == '1'):
		print('Playing bass')
		#playsound('./sound/Bass.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '2'):
		print('Playing Snare')
		playsound('./sound/Snare.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '3'):
		playsound('./sound/Tom1.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '4'):
		playsound('./sound/Tom2.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '5'):
		playsound('./sound/Hihat.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '6'):
		playsound('./sound/Bass.mp3',False)
		playsound('./sound/Snare.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '7'):
		playsound('./sound/Bass.mp3',False)
		playsound('./sound/Tom1.mp3')
		r.set('drum_key', '0')

	if(r.get('drum_key') == '8'):
		playsound('./sound/Bass.mp3',False)
		playsound('./sound/Tom2.mp3')
		r.set('drum_key', '0')						
		

	
	
		
