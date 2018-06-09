#!/usr/bin/env python
#encoding: utf-8
import rospy, subprocess, os, datetime
from std_msgs.msg import String

def open_jtalk(t):
	open_jtalk=['open_jtalk']
	mech=['-x','/var/lib/mecab/dic/open-jtalk/naist-jdic']
	htsvoice=['-m', '/usr/share/hts-voice/nitech-jp-atr503-m001/nitech_jp_atr503_m001.htsvoice']
	
	speed=['-r', '1.0']
	out_wav=['-ow', 'julius_open_jtalk.wav']
	out_log=['-ot', 'julius_open_jtalk.log']
	cmd=open_jtalk+mech+htsvoice+out_wav+out_log+speed
	c=subprocess.Popen(cmd, stdin=subprocess.PIPE)
	c.stdin.write(t)
	c.stdin.close()
	c.wait()
	aplay=['aplay', 'julius_open_jtalk.wav']
	wr=subprocess.Popen(aplay)

	
def callback(data):
	pub = rospy.Publisher('/speech_word', String, queue_size=100)
	speech_word = data.data
	print speech_word
	open_jtalk(speech_word)	
	pub.publish(String(speech_word))
	
def julius_subscriber():
	rospy.init_node('julius_subscriber', anonymous=True)
	rospy.Subscriber("julius_voice_string", String, callback)
	rospy.spin()
	

if __name__ == '__main__':
	julius_subscriber()
	os.system("rm julius_open_jtalk.wav")
