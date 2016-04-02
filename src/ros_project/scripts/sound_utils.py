#!/usr/bin/env python
import rospy
from Queue import Queue
from threading import Thread
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class SoundUtils():

    INSTANCE = None

    @classmethod
    def get_instance(cls):
        if cls.INSTANCE is None:
            cls.INSTANCE = SoundUtils()
        return cls.INSTANCE

    def __init__(self):
        if self.INSTANCE is not None:
            raise ValueError("An instantiation already exists!")
        # rospy.init_node('say', anonymous = True)
        self.voice = ''
        self.volume = 1.0
        self.soundhandle = SoundClient()
        self.last_message = ''

        
        self.last_time_said = rospy.Time(0)
        self.time_sleep_in_sec = rospy.Time(2).to_sec()
        self.time_delay_in_sec = rospy.Time(30).to_sec()
        self.previous_messages = {}
        self.message_queue = Queue()
        self.thread =  Thread(target=self.observe_queue)
        self.thread.start()
        # self._trigger_cond = threading.Condition()
        # rospy.sleep(1)

    def observe_queue(self):
        while True:
            if not self.message_queue.empty():
                self.soundhandle.say(self.message_queue.get())
            rospy.sleep(self.time_sleep_in_sec)
            pass

    def say_message(self,message):
        # if message == self.last_message:
            # return
        now = rospy.Time().now()
        if message in self.previous_messages:
            last_time_said = self.previous_messages[message]
            diff_in_sec = (now - last_time_said).to_sec()
            if diff_in_sec < self.time_delay_in_sec:
                return
        rospy.loginfo(message)
        self.previous_messages[message] = now
        self.message_queue.put(message)

    def say_message2(self,message):
        # if message == self.last_message:
            # return

        now = rospy.Time().now()
        diff_in_sec = (now - self.last_time_said).to_sec()
        if diff_in_sec < self.time_sleep_in_sec:
            return

        self.last_message = message
        self.last_time_said = now
        self.soundhandle.say(message, self.voice, self.volume)



# if __name__ == '__main__':
#     try:
#         SoundUtils()
#         rospy.sleep(1)
#     except:
#         pass

