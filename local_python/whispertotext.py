import os
import wave
import numpy as np
import sounddevice as sd
from pydub import AudioSegment
import sys
import select
import termios

'''
This class was developed to simplify use of OpenAI whisper model (speech to text).
Methods:
    record_audio -> starts to record a message when user hits enter and stops on second enter press.
    split_audio_if_needed -> segments audio into chunks for transcription based on the size of the files (whisper has a size limit)
    transcribe_audio_chunks -> provides a transcription for a series of audio file chunks
    confirm_transcription -> method to display the full transcription and have user verify in the terminal  
'''

class Speech2Text:
    def __init__(self, client, prompt=None, sample_rate=44100, channels=1, max_file_size_mb=25):
        self.client = client
        self.prompt = prompt
        self.sample_rate = sample_rate
        self.channels = channels
        self.max_file_size_mb = max_file_size_mb

    def record_audio(self, filename):
        print("Press Enter to start recording...")
        input()  # Wait for Enter to start
        print("Starting recording... Press Enter again to stop.")

        frames = []
        try:
            with sd.InputStream(samplerate=self.sample_rate, channels=self.channels, dtype='int16') as stream:
                while True:
                    frame, overflowed = stream.read(self.sample_rate // 10)  # Read in small chunks
                    frames.append(frame)
                    if self._check_for_enter(clear_buffer=False):  # Do not consume stdin buffer
                        break
        except KeyboardInterrupt:
            print(" Recording stopped by user.")
        print(" Stopping recording...")

        # Concatenate frames and write to file
        if len(frames) > 0:
            recording = np.concatenate(frames, axis=0)
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(recording.tobytes())
            print(f" Audio saved to {filename}")
        else:
            print("[ERROR] No audio recorded. Please try again.")


        # Concatenate frames and write to file
        if len(frames) > 0:
            recording = np.concatenate(frames, axis=0)
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(self.channels)
                wf.setsampwidth(2)
                wf.setframerate(self.sample_rate)
                wf.writeframes(recording.tobytes())
            print(f" Audio saved to {filename}")
        else:
            print("[ERROR] No audio recorded. Please try again.")

    def split_audio_if_needed(self, audio_file_path):
        audio = AudioSegment.from_wav(audio_file_path)
        file_size_mb = len(audio.raw_data) / (1024 * 1024)
        chunks = []
        if file_size_mb > self.max_file_size_mb:
            print(f" Audio file size ({file_size_mb:.2f} MB) exceeds {self.max_file_size_mb} MB. Splitting into smaller chunks...")
            start_time = 0
            while start_time < len(audio):
                end_time = min(start_time + (self.max_file_size_mb * 60 * 1000 / file_size_mb) * 1000, len(audio))
                chunk = audio[start_time:end_time]
                chunk_filename = os.path.join("recordings", f"chunk_{start_time // 1000}-{end_time // 1000}.wav")
                chunk.export(chunk_filename, format="wav")
                chunks.append(chunk_filename)
                start_time = end_time
        else:
            chunks.append(audio_file_path)
        return chunks

    def transcribe_audio_chunks(self, chunks):
        full_transcription = ""
        for chunk_file in chunks:
            print(f" Transcribing chunk: {chunk_file}")
            with open(chunk_file, "rb") as audio_file:
                transcription = self.client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    prompt=self.prompt
                )
                full_transcription += transcription.text + " "
        return full_transcription.strip()
    
    def confirm_transcription(self, transcription):
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        try:
            response = input(f"Is the transcription correct? (yes/no): '{transcription}' ").strip().lower()
            if response == "yes":
                return True
            elif response == "no":
                return False
            else:
                print("Invalid response. Please type 'yes' or 'no'.")
        except EOFError:
            print("Unexpected input error. Please try again.")

    @staticmethod
    def _check_for_enter(clear_buffer=True):

        i, _, _ = select.select([sys.stdin], [], [], 0.1)
        if i:
            if clear_buffer:
                sys.stdin.readline()  # Clear the buffer
            return True
        return False


