import matplotlib.pyplot as plt
import csv

file1 = 'full_pitch.csv'
file2 = 'half_pitch.csv'
file3 = 'zero_pitch.csv'

def read_csv(filepath):
    x = []
    y = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        next(reader)
        for row in reader:
            x.append(float(row[0]))
            y.append(float(row[1]))
    return x, y

x1, y1 = read_csv(file1)
x2, y2 = read_csv(file2)
x3, y3 = read_csv(file3)

plt.figure(figsize=(10, 6))
plt.plot(x1, y1, label='100% pitch', color='red')
plt.plot(x2, y2, label='50% pitch', color='green')
plt.plot(x3, y3, label='0% pitch', color='blue')

plt.title('Duty Cyle vs Time (Fixed Throttle: 100%)')
plt.xlabel('Time (s)')
plt.ylabel('Pulse width (OneShot125)')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
