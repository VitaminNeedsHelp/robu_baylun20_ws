import random

numbers = [random.randint(-10, 10) for i in range(10)]
print(f"Zufällige Zahlen: {numbers}")
print(f"Summe: {sum(numbers)}")
print(f"Minimum: {min(numbers)}")
print(f"Maximum: {max(numbers)}")
print(f"Mittlerer Wert: {sum(numbers) / len(numbers)}")