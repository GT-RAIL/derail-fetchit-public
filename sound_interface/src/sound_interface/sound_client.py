#!/usr/bin/env python
# Interface to sounds for Ava

from __future__ import print_function, division

import os
import psutil
import argparse
import requests
import Queue
import numpy as np

from pydub import AudioSegment

import rospy
import rospkg
import actionlib

from sound_play.msg import SoundRequestAction, SoundRequestGoal, SoundRequest

# Helper functions

FILENAME_CHARS = list("abcdefghijklmnopqrstuvwxyz0123456789_")
FILENAME_NUM_RANDOM_CHARS = 10


def create_temp_filename(prefix, suffix):
    """A temporary filename creation helper. It's a bit of a hack"""
    global FILENAME_CHARS
    global FILENAME_NUM_RANDOM_CHARS
    name = "{}{}{}".format(
        prefix,
        "".join(np.random.choice(FILENAME_CHARS, FILENAME_NUM_RANDOM_CHARS)),
        suffix
    )
    filename = "/tmp"
    while os.path.exists(filename):
        filename = os.path.join("/tmp", name)
    return filename


# The actual sound client

class SoundClient(object):
    """
    A reimplementation of ROS's SoundClient to help with speech and sound
    tasks. We force the use of an action client in this class.

    To play a beep, you must refer to one of the constant keys in this
    package that begins with `SOUND_*`. By default, the sound files in `sounds`
    map to one of these keys.

    To speak, you can use SSML syntax to specify text. The TTS interface
    expects a MaryTTS server running in the background. The TTS could be a long
    running process and is currently blocking.
    """

    # Template SSML
    SSML_TEMPLATE = \
    """<?xml version="1.0" encoding="UTF-8" ?>
<speak version="1.0" xmlns="http://www.w3.org/2001/10/synthesis"
  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
  xsi:schemaLocation="http://www.w3.org/2001/10/synthesis http://www.w3.org/TR/speech-synthesis/synthesis.xsd"
  xml:lang="en-US">
{speech}
</speak>
    """

    # Keys for the different beeps
    BEEP_CHEERFUL = "CHEERFUL"
    BEEP_CONCERNED = "CONCERNED"
    BEEP_EXCITED = "EXCITED"
    BEEP_HAPPY = "HAPPY"
    BEEP_PLAYFUL = "PLAYFUL"
    BEEP_PROUD = "PROUD"
    BEEP_SAD = "SAD"
    BEEP_SHOCKED = "SHOCKED"
    BEEP_SURPRISED = "SURPRISED"
    BEEP_UNSURE = "UNSURE"

    # Keys for the different affects
    AFFECT_SAD = "SAD"
    AFFECT_HAPPY = "HAPPY"
    AFFECT_ANGRY = "ANGRY"
    AFFECT_CALM = "CALM"
    AFFECT_NERVOUS = "NERVOUS"

    # Mary TTS server URL
    MARY_SERVER_URL = "http://localhost:59125/process"
    MARY_SERVER_TIMEOUT = 30  # Number of seconds to wait before timing out

    # Speech params
    SPEECH_GAIN_DB = 15

    # ROS Constants
    SOUND_PLAY_SERVER = "/sound_server"

    @staticmethod
    def make_happy(text):
        """Make the text happy!"""
        # return ('<emotion><category name="happy" />{}</emotion>'.format(text))
        return text

    @staticmethod
    def make_sad(text):
        """Make the text sad :("""
        # return ('<emotion><category name="sad" />{}</emotion>'.format(text))
        return text

    @staticmethod
    def make_angry(text):
        """Make the text angry"""
        # return ('<emotion><category name="angry" />{}</emotion>'.format(text))
        return text

    @staticmethod
    def make_calm(text):
        """Make the text calm"""
        # return (
        #     """
        # <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">
        #     {}
        #     <dimension name="arousal" value="0.3"/><!-- lower arousal -->
        #     <dimension name="pleasure" value="0.9"/><!-- high positive valence -->
        #     <dimension name="dominance" value="0.8"/><!-- high potency    -->
        # </emotion>
        #     """.format(text)
        # )
        return text

    @staticmethod
    def make_nervous(text):
        """Make the text nervous"""
        # return (
        #     """
        # <emotion dimension-set="http://www.w3.org/TR/emotion-voc/xml#pad-dimensions">
        #     {}
        #     <dimension name="arousal" value="0.9"/><!-- high arousal -->
        #     <dimension name="pleasure" value="0.2"/><!-- negative valence -->
        #     <dimension name="dominance" value="0.2"/><!-- low potency    -->
        # </emotion>
        #     """.format(text)
        # )
        return text

    @staticmethod
    def change_audio_speed(sound, speed=1.0):
        # Manually override the frame_rate. This tells the computer how many
        # samples to play per second
        sound_with_altered_frame_rate = sound._spawn(sound.raw_data, overrides={
            "frame_rate": int(sound.frame_rate * speed)
        })

        # convert the sound with altered frame rate to a standard frame rate
        # so that regular playback programs will work right. They often only
        # know how to play audio at standard frame rate (like 44.1k)
        return sound_with_altered_frame_rate.set_frame_rate(sound.frame_rate)

    def __init__(self, beeps=None, affects=None):
        # Want to reimplement SoundClient so that we are always using the action
        # interface to the sound_play_node.
        self.sound_client = actionlib.SimpleActionClient(
            SoundClient.SOUND_PLAY_SERVER, SoundRequestAction
        )

        # Background thread to clear out speech files when they're done
        self._tmp_speech_files = Queue.Queue()
        self._tmp_speech_cleanup_thread = rospy.Timer(rospy.Duration(60), callback=self._cleanup)

        # Load the beeps
        self.beeps = beeps
        if self.beeps is None:
            default_sound_path = os.path.join(
                rospkg.RosPack().get_path('sound_interface'),
                'sounds'
            )
            self.beeps = {
                SoundClient.BEEP_CHEERFUL: os.path.join(default_sound_path, 'R2D2_cheerful.wav'),
                SoundClient.BEEP_CONCERNED: os.path.join(default_sound_path, 'R2D2_concerned.wav'),
                SoundClient.BEEP_EXCITED: os.path.join(default_sound_path, 'R2D2_excited.wav'),
                SoundClient.BEEP_HAPPY: os.path.join(default_sound_path, 'R2D2_happy.wav'),
                SoundClient.BEEP_PLAYFUL: os.path.join(default_sound_path, 'R2D2_playful.wav'),
                SoundClient.BEEP_PROUD: os.path.join(default_sound_path, 'R2D2_proud.wav'),
                SoundClient.BEEP_SAD: os.path.join(default_sound_path, 'R2D2_sad.wav'),
                SoundClient.BEEP_SHOCKED: os.path.join(default_sound_path, 'R2D2_shocked.wav'),
                SoundClient.BEEP_SURPRISED: os.path.join(default_sound_path, 'R2D2_surprised.wav'),
                SoundClient.BEEP_UNSURE: os.path.join(default_sound_path, 'R2D2_unsure.wav'),
            }

        # Load the affects
        self.affects = {
            SoundClient.AFFECT_SAD: SoundClient.make_sad,
            SoundClient.AFFECT_HAPPY: SoundClient.make_happy,
            SoundClient.AFFECT_NERVOUS: SoundClient.make_nervous,
            SoundClient.AFFECT_CALM: SoundClient.make_calm,
            SoundClient.AFFECT_ANGRY: SoundClient.make_angry,
        }

        # Need to connect to the server
        rospy.loginfo("Connecting to {}...".format(SoundClient.SOUND_PLAY_SERVER))
        self.sound_client.wait_for_server()
        rospy.loginfo("...{} connected".format(SoundClient.SOUND_PLAY_SERVER))

    def get_state(self):
        """Returns the state of the action client"""
        return self.sound_client.get_state()

    def get_result(self, blocking=False):
        """Returns the result of the last sound action. Blocks for a result"""
        if blocking:
            self.sound_client.wait_for_result()
        return self.sound_client.get_result()

    def get_beep_names(self):
        """Get the keys to the different beep types that are available"""
        return self.beeps.keys()

    def get_affect_names(self):
        """Get the keys to the different affects that are available"""
        return self.affects.keys()

    def say(self, text, affect="", blocking=False, **kwargs):
        """Perform TTS using SSML"""

        # Transform the text if the affect argument calls for it
        if affect and affect.upper() in self.affects.keys():
            text = self.affects[affect.upper()](text)

        # Create the vars for the SSML query
        text = SoundClient.SSML_TEMPLATE.format(speech=text)
        query_dict = {
            'INPUT_TEXT': text,
            'INPUT_TYPE': 'SSML',
            'LOCALE': 'en_GB',
            'VOICE': 'dfki-prudence-hsmm',
            'OUTPUT_TYPE': 'AUDIO',
            'AUDIO': 'WAVE',
            # 'effect_Robot_selected': 'on',
            # 'effect_Robot_parameters': 'amount:60.0',
        }

        # Send a request to MARY and check the response type
        r = requests.post(
            SoundClient.MARY_SERVER_URL,
            params=query_dict,
            timeout=SoundClient.MARY_SERVER_TIMEOUT
        )
        if r.headers['content-type'] != 'audio/x-wav':
            rospy.logerr("Response Error Code: {}. Content-Type: {}"
                         .format(r.status_code, r.headers['content-type']))
            raise ValueError("Incorrect Content Type", r.headers['content-type'], r.status_code)

        # Increase the volume on the temp file
        speech = AudioSegment(data=r.content)
        speech = speech + SoundClient.SPEECH_GAIN_DB
        speech = SoundClient.change_audio_speed(speech, 0.95)
        speech = speech.set_frame_rate(int(speech.frame_rate*2.0))

        # Write the wav data to a temp file
        speech_filename = create_temp_filename(prefix='marytts', suffix='.wav')
        with open(speech_filename, 'wb') as fd:
            speech.export(speech_filename, format='wav')

        # Now send the file's name over to sound play
        sound = SoundRequest()
        sound.sound = SoundRequest.PLAY_FILE
        sound.command = SoundRequest.PLAY_ONCE
        sound.arg = speech_filename
        self._play(sound, blocking=blocking, **kwargs)

        # Send the file to the cleanup thread now
        self._tmp_speech_files.put(speech_filename)

    def beep(self, key, blocking=False, **kwargs):
        """Play one of the beeps and boops that we know of"""
        if not key or key.upper() not in self.beeps:
            return

        sound = SoundRequest()
        sound.sound = SoundRequest.PLAY_FILE
        sound.command = SoundRequest.PLAY_ONCE
        sound.arg = self.beeps[key.upper()]
        self._play(sound, blocking=blocking, **kwargs)

    def stop(self):
        """Stop all sounds"""
        self.sound_client.cancel_all_goals()

        # Send a stop request
        sound = SoundRequest()
        sound.sound = SoundRequest.ALL
        sound.command = SoundRequest.PLAY_STOP
        sound.arg = ""

        self._play(sound, blocking=True)

    def _play(self, sound, blocking, **kwargs):
        # Send the command
        rospy.logdebug("Sending sound action with (sound, command, arg): {}, {}, {}"
                       .format(sound.sound, sound.command, sound.arg))
        goal = SoundRequestGoal(sound_request=sound)
        self.sound_client.send_goal(goal)

        # If blocking, wait until the sound is done
        if blocking:
            self.sound_client.wait_for_result()
            rospy.logdebug('Response to sound action received')

    def _cleanup(self, evt):
        # Get the files that we want to check are safe to delete or not
        files_to_check = set()
        while not self._tmp_speech_files.empty():
            files_to_check.add(self._tmp_speech_files.get(block=False))
            self._tmp_speech_files.task_done()

        # Iterate through all the processes and check if they are using
        # any of the files
        files_to_leave = set()
        for proc in psutil.process_iter():
            try:
                for filename in files_to_check:
                    if filename in [x.path for x in proc.open_files()]:
                        rospy.logdebug("File is used by: {}".format(proc.info))
                        files_to_leave.add(filename)
                        self._tmp_speech_files.put(filename)
            except Exception as e:
                pass

        # Finally delete only the files that we know we should leave
        for filename in (files_to_check - files_to_leave):
            if os.path.exists(filename):
                rospy.logdebug("Removing temp speech file: {}".format(filename))
                os.remove(filename)


if __name__ == '__main__':
    # For testing purposes
    rospy.init_node('speaker_test')
    SoundClient.SOUND_PLAY_SERVER = "sound_play"
    client = SoundClient()

    def on_speak(args):
        text = args.text
        affect = args.affect
        client.say(text, affect, blocking=True)

    def on_beep(args):
        client.beep(args.key, blocking=True)

    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers()

    speak_parser = subparsers.add_parser("speak", help="get the robot to speak")
    speak_parser.add_argument("text", help="text to speak")
    speak_parser.add_argument("--affect", choices=client.get_affect_names(),
                              help="say things in an affected voice")
    speak_parser.set_defaults(func=on_speak)

    beep_parser = subparsers.add_parser("beep", help="get the robot to beep")
    beep_parser.add_argument("key", help="type of beep to produce",
                             choices=client.get_beep_names())
    beep_parser.set_defaults(func=on_beep)

    args = parser.parse_args()
    args.func(args)
