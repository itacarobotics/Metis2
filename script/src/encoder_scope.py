import serial
import matplotlib.pyplot as plt
import numpy as np

def main():
    # Open serial port
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)

    # Create a dynamic plot
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots()
    x_len = 100  # Number of points to display
    y_data = [0] * x_len  # Y-axis data (encoder positions)
    x_data = list(range(x_len))  # X-axis data (time points)
    line, = ax.plot(x_data, y_data)
    
    ax.set_title("Encoder position angle")
    ax.set_ylim(-360, 360)  # Adjust based on the range of your encoder values
    ax.grid(True)

    # Add a text object to display the current encoder value
    text = ax.text(0.7, 0.9, '', transform=ax.transAxes, fontsize=12, color='red')


    while True:
        try:
            # Read 4 bytes from serial (encoder position)
            msg = ser.read(4)
            encoder_value = int.from_bytes(msg, byteorder='little', signed=True)
            encoder_value = (encoder_value / 2000) * 360


            # Update the plot with new data
            y_data.append(encoder_value)  # Add new encoder position
            y_data.pop(0)  # Remove the oldest point

            # Update line with new data
            line.set_ydata(y_data)
            
            # Update text with the current encoder value
            text.set_text(f'Angle: {round(encoder_value,1)} [deg]')
            
            # Redraw the plot
            fig.canvas.draw()
            fig.canvas.flush_events()
            
        except KeyboardInterrupt:
            print("Exiting...")
            break

    ser.close()


if __name__ == "__main__":
    main()
