import speech_recognition as sr
import pyttsx3

# Initialize the text-to-speech engine
engine = pyttsx3.init()

# Define the function to sort tools
def sort_tools():
    print("Sorting tools...")
    engine.say("I am sorting the tools.")
    engine.runAndWait()

# Listen for speech input
def listen_for_command():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening...")
        try:
            audio = recognizer.listen(source, timeout=5)
            command = recognizer.recognize_google(audio)
            return command.lower()
        except sr.WaitTimeoutError:
            print("No speech detected.")
        except sr.UnknownValueError:
            print("Couldn't understand what you said.")
        except sr.RequestError as e:
            print(f"Speech recognition service error: {e}")
    return ""

# Main program
def main():
    while True:
        print("Waiting for activation word...")
        activation = listen_for_command()
        
        if "kobot" in activation:
            print("Kobot activated! Waiting for your command...")
            engine.say("I am ready. What would you like me to do?")
            engine.runAndWait()
            
            command = listen_for_command()
            
            if "sort the tools" in command:
                sort_tools()
            else:
                print("Unknown command.")
                engine.say("I didn't understand the command.")
                engine.runAndWait()

# Run the main loop
if __name__ == "__main__":
    main()
