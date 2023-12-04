import tkinter as tk
import time

class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=1000, height=1000)
        self.canvas.pack()
        self.canvas.create_text(50, 20, text="Ground Truth", fill="green")
        self.canvas.create_text(50, 40, text="Correction", fill="blue")
        self.canvas.create_text(50, 60, text="Prediction", fill="red")
        self.rectangles_on_canvas = []

    def plot_position(self, position, position_type):
        x, y = position
        # transform position to canvas coordinates so 0,0 is in the middle of the canvas
        x = x * 50 + 500
        y = y * -50 + 500

        if position_type == "prediction":
            color = "red"
        elif position_type == "correction":
            color = "blue"
        elif position_type == "ground truth":
            color = "green"
        else:
            print(position_type)
            raise ValueError("Invalid position type")

        rec = self.canvas.create_rectangle(x - 20, y - 20, x + 20, y + 20, outline=color)
        
        self.rectangles_on_canvas.append(rec)
        
        if len(self.rectangles_on_canvas) > 3:
            self.canvas.delete(self.rectangles_on_canvas.pop(0))

    def plot_positions_from_file(gui):
                with open("gui.txt", "r") as file:
                    lines = file.readlines()
                counter = 0
                for line in lines:
                    values = line.strip().split()
                    if counter % 2 == 0:
                        x_p = values[1]
                        y_p = values[2]
                        x_a = values[3]
                        y_a = values[4]
                        gui.plot_position((float(x_p), float(y_p)), "prediction")
                        gui.plot_position((float(x_a), float(y_a)), "ground truth")
                        time.sleep(0.15)
                    else:
                        x_c = values[1]
                        y_c = values[2]
                        gui.plot_position((float(x_c), float(y_c)), "correction")
                        time.sleep(0.05)
                    gui.root.update()
                    counter += 1
                        

    def run(self):
        self.root.mainloop()

# Example usage
gui = GUI()
gui.plot_positions_from_file()
gui.run()
