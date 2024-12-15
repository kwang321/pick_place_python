
import speech_recognition as sr
import subprocess

recognizer = sr.Recognizer()

# Record audio using `arecord`
print("Recording audio...")
subprocess.run(["arecord", "-d", "5", "-f", "cd", "-t", "wav", "test.wav"])
print("Recording complete. Recognizing...")

# Play back the recorded audio using `aplay`
print("Playing back the recorded audio...")
subprocess.run(["aplay", "test.wav"])

# Process the audio file
with sr.AudioFile("test.wav") as source:
    audio = recognizer.record(source)

try:
    # Recognize speech using Google API
    text = recognizer.recognize_google(audio)
    print(f"You said: {text}")
except sr.RequestError:
    print("Could not request results from Google Speech Recognition service")
except sr.UnknownValueError:
    print("Could not understand the audio")

