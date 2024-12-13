import tkinter as tk

# Function to handle button click
def on_button_click(event):
    # Change to a "pressed" state
    canvas.itemconfig(button_inner, fill="lightgreen")
    canvas.itemconfig(button_outer, fill="darkgreen")
    root.after(200, reset_button)  # Reset the button after 200ms

    # Display the loading scene
    display_loading_scene()

    # Start the loading animation
    animate_loading_line()

# Function to reset the button appearance
def reset_button():
    canvas.itemconfig(button_inner, fill="green")
    canvas.itemconfig(button_outer, fill="forestgreen")

# Function to display the loading scene text
def display_loading_scene():
    loading_label.config(text="Your robot is sorting the tools...")

# Function to animate the loading line
def animate_loading_line():
    # Move the line horizontally across the canvas
    for i in range(0, 300, 5):
        canvas.after(i * 10, move_loading_line, i)

# Function to move the loading line
def move_loading_line(x_position):
    # Redraw the line at the new position
    canvas.coords(loading_line, 0, 270, x_position, 270)  # Move it down by changing the y-coordinates

    # Restart the animation when the line reaches the end
    if x_position >= 300:
        canvas.after(500, restart_loading_animation)

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

