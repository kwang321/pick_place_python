import tkinter as tk
from pick_place_python import MoveGroupPythonInterfaceTutorial
import threading
import time
# Initialize the interface
tutorial = MoveGroupPythonInterfaceTutorial()
tutorial.setup_scene()
tutorial.add_tools()

# Flag to prevent multiple button clicks
button_active = False
animation_id = None

# Function to reset the button appearance
def reset_button():
    global button_active
    canvas.itemconfig(button_inner, fill="green")
    canvas.itemconfig(button_outer, fill="forestgreen")
    button_active = False  # Allow button to be pressed again

# Function to handle button click
def on_button_click(event):
    global button_active
    if button_active:  # Check if button is already active
        return

    # Set the button as active
    button_active = True

    # Change to a "pressed" state
    canvas.itemconfig(button_inner, fill="lightgreen")
    canvas.itemconfig(button_outer, fill="darkgreen")
    root.after(200, reset_button)

    # Display the loading scene
    display_loading_scene()

    # Start the loading animation
    animate_loading_line()

    # Start the sorting process in a separate thread
    threading.Thread(target=sort_tools_and_reset).start()


# Function to display the loading scene text
def display_loading_scene():
    loading_label.config(text="Your robot is sorting the tools...")


animation_id = None

# Function to animate the loading line
def animate_loading_line():
    move_line(0)  # Start moving the line from position 0

# Function to move the loading line
def move_line(x_position):
    # Redraw the line at the new position
    canvas.coords(loading_line, x_position, 270, x_position + 30, 270)  # Draw a short line, moving to the right

    # Check if the line has reached the right edge (300px in this case)
    if x_position >= 300:
        # Reset the position back to the left side and start the movement again
        canvas.after(0, move_line, -30)  # Start from the left again
    else:
        # Move the line to the right
        global animation_id
        animation_id = canvas.after(10, move_line, x_position + 1)  # Move 5 pixels to the right every 10ms

# Function to stop the loading animation and update the text
def stop_loading_animation():
    # Cancel the scheduled animation
    global animation_id
    if animation_id:
        canvas.after_cancel(animation_id)
        animation_id = None

    canvas.delete(loading_line)

    
    # Change the displayed text
    loading_label.config(text="Your tools are sorted!")

# Function to run the sort_tools() and reset the button after it's done
def sort_tools_and_reset():
    # Call the sorting operation
    global done
    done = tutorial.sort_tools()

    if done:
        stop_loading_animation()
        reset_button()




# Function to restart the animation
def restart_loading_animation():
    canvas.coords(loading_line, 0, 270, 0, 270)  # Reset the line position
    animate_loading_line()  # Start the animation again

# Create the main window
root = tk.Tk()
root.title("Robot assistant")

# Create a canvas to draw the button and animation
canvas = tk.Canvas(root, width=300, height=300, bg="white")
canvas.pack()

# Add text above the button
canvas.create_text(150, 50, text="Press the Button", font=("Arial", 16), fill="black")

# Draw the outer and inner circles for the 3D effect
button_outer = canvas.create_oval(75, 75, 225, 225, fill="forestgreen", outline="black", width=3)
button_inner = canvas.create_oval(85, 85, 215, 215, fill="green", outline="black", width=2)

# Add text in the middle of the button
canvas.create_text(150, 150, text="Sort", font=("Arial", 14, "bold"), fill="white")

# Add click behavior to the button
canvas.tag_bind(button_outer, "<Button-1>", on_button_click)
canvas.tag_bind(button_inner, "<Button-1>", on_button_click)

# Add a label for the loading scene
loading_label = tk.Label(root, text="", font=("Arial", 14), fg="blue", bg="white")
loading_label.pack(pady=10)

# Draw the loading line (initially positioned lower)
loading_line = canvas.create_line(0, 270, 0, 270, width=4, fill="blue")

# Run the Tkinter event loop
root.mainloop()


