#!/usr/bin/env python

from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
import Queue
import rospy
from std_msgs.msg import String


class GspeechClient(object):
    def __init__(self):
        # Audio stream input setup
    
        #Shut down command
        rospy.on_shutdown(self.shutdown)
        
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        self.CHUNK = 4096
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, channels=CHANNELS,
                                      rate=RATE, input=True,
                                      frames_per_buffer=self.CHUNK,
                                      stream_callback=self._get_data)
        self._buff = Queue.Queue()  # Buffer to hold audio data
        self.closed = False

        # ROS Text Publisher
        text_topic = rospy.get_param('/text_topic','/text')
        text2_topic = rospy.get_param('/text2_topic', '/intent')
        self.text_pub = rospy.Publisher(text_topic, String, queue_size=10)
        self.text2_pub = rospy.Publisher(text2_topic, String, queue_size=10)

    def _get_data(self, in_data, frame_count, time_info, status):
        """Daemon thread to continuously get audio data from the server and put
         it in a buffer.
        """
#         Uncomment this if you want to hear the audio being replayed.
        self._buff.put(in_data)
        return None, pyaudio.paContinue

    def _generator(self):
        """Generator function that continuously yields audio chunks from the buffer.
        Used to stream data to the Google Speech API Asynchronously.
        """
        while not self.closed:
            # Check first chunk of data
            chunk = self._buff.get()
            if chunk is None:
                return
            data = [chunk]

            # Read in a stream till the end using a non-blocking get()
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    data.append(chunk)
                except Queue.Empty:
                    break

            yield b''.join(data)

    def _listen_print_loop(self, responses):
        """Iterates through server responses and prints them.
        The responses passed is a generator that will block until a response
        is provided by the server.
        Each response may contain multiple results, and each result may contain
        multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
        print only the transcription for the top alternative of the top result.
        """
        for response in responses:
            # If not a valid response, move on to next potential one
            if not response.results:
                continue

            # The `results` list is consecutive. For streaming, we only care about
            # the first result being considered, since once it's `is_final`, it
            # moves on to considering the next utterance.
            result = response.results[0]
            if not result.alternatives:
                continue

            # Display the transcription of the top alternative.
            transcript = result.alternatives[0].transcript

            # Parse the final utterance
            if result.is_final:
                rospy.loginfo("Google Speech result: {}".format(result))
                # Received data is Unicode, convert it to string
                transcript = transcript.encode('utf-8')
                # Strip the initial space, if any
                if transcript.startswith(' '):
                    transcript = transcript[1:]
                # Exit if needed
                if transcript.lower() == 'exit':
                    self.shutdown()
                # Send the rest of the sentence to topic
                self.text_pub.publish(transcript)
                #Checks if sentence contains command in dictionary
                commands = ['microwave', 'bathroom']
                mycommands = transcript.split(' ')
                check_command = False
                for i in commands:
                    for j in mycommands:
                        if i == j:
                            print 'Sure, going to %s' %(j)
                            check_command = True
                            rospy.loginfo(j)
                            self.text2_pub.publish(j)
                            if check_command == False:
                                print "Sorry, cant do that"
                                break
#                rospy.sleep(3)
                
    

    def gspeech_client(self):
        """Creates the Google Speech API client, configures it, and sends/gets
        audio/text data for parsing.
        """
        language_code = 'en-US'
        client = speech.SpeechClient()
        config = types.RecognitionConfig(
            encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=16000,
            language_code=language_code)
        streaming_config = types.StreamingRecognitionConfig(
            config=config,
            interim_results=True,
            single_utterance=True)
        requests = (types.StreamingRecognizeRequest(audio_content=content) for content in self._generator())
        responses = client.streaming_recognize(streaming_config, requests)
        self._listen_print_loop(responses)

    def shutdown(self):
        """Shut down as cleanly as possible"""
        rospy.loginfo("Shutting down")
        self.closed = True
        self._buff.put(None)
        self.stream.close()
        self.audio.terminate()
        exit()

    def start_client(self):
        """Entry function to start the client"""
        try:
            rospy.loginfo("Starting Google speech mic client")
            self.gspeech_client()
        except KeyboardInterrupt:
            self.shutdown()


if __name__ == '__main__':
    rospy.init_node('mic_client')
    g = GspeechClient()
    g.start_client()

