#Kommentare: Hashtag erzeugt einen einzeiligen Kommentar 
# Python Interpreter starten über CLI starten 
# Python Programm starten über CLI: python3 dateipfad.py


from math import pi as M
import matplotlib.pyplot as plt # Bibliothek zum Erzeugen von X-Y Plots 

#Variablennamen 
#muessen mit Buchstaben/Underscore beginnen (keine Zahlen)
#case sensitive 

titel = "Datentypen" #wird als str erkannt 
vorname = "Lukas"
nachname = "Bayer"
name = vorname + " " + nachname     #str
klassenvorstand = False             #bool
direktor = False 
groesse = 1.85                      #float
schuhgroesse = 46                   #int
impedanz = 10 + 20j                 #complex

# print("Aausgabe von unterschiedlichen", titel)
print(f"Ausgabe von unterschiedlichen {3*(titel + ' ')}")
print(f"Mein Name ist {name}")

print(klassenvorstand, type(klassenvorstand))
print(groesse, type(groesse))
print(schuhgroesse, type(schuhgroesse))
print(impedanz, type(impedanz))

print("Impedanzberechnung von einem RC-Glied")
print("=====================================")

resistor = 1_000         #1000 = 1_000 = 1e3
capacitor = 1e-6        #F
frequency = 50.0        #Hz

impedanz = complex(resistor, 1/(2*M*frequency*capacitor))
tau = resistor * capacitor #Zeitkonstante 

print(f"Impedanz: {impedanz/1000:<10.2f} kOhm")
print("Zeitkonstante: {1000 * tau:^0.2f} ms")
print("Grenzgrequenz: {1/2(*M*tau) :<10.2f} Hz")

#Datentyp List - listen (muteable)

print("Datentyp list")
print("=============")

myList = [1, 2, -5, 3.14, "some text", [3,2], name, impedanz]
print(myList[0])
print(myList[-1])           #kommt zum letzten Element 
print(myList[-3] [0])       #gibt inhalt der zweiten Liste aus 
print(myList[2:4])          #slicing Ausgabe von einem bestimmten Bereich 
print(myList[-2:])          #Ausgabe der letzten zwei Elemente 
print(myList[:5:2])         #gibt jedes zweite Element aus
                            #beginnen von 0 bis einschließlich 5

print(myList[::2])          #gibt jede gearde Elemente mit einen geraden Index aus 

myList += [[]]            #fügt am Ende eine Leere Liste hinzu 
myList.append(3)            #fügt am ende eine 3 hinzu 
print(myList)


#Entpacken von Listen 
#=========================
print("Entpacken von Listen")
print("====================")

a, b, c = myList[: 3]
print(a,b,c)

#Datentypen set 
#=========================
print("Datentyp set")
print("============")
x = [1, 4, 4, 6, 7, 8, 1, 6]
y = {1, 5, 4, 6, 7, 5, 8, 4}
x.sort()
x_set = set(x)
print("x: ", x)
print("x_set: ", x_set)

#dictioniaries
#=========================

print("Datentyp dict")
print("=============")

person = {"vorname": vorname,
        "nachname": nachname,
        "groesse": groesse, 
        "kv": klassenvorstand,
        "zustand": "gesund"}

print(person["vorname"],
      person["nachname"],
      "ist unser klassenvorstand und sieht",
      person["zustand"], "aus.")

print("person.keys(): ", person.keys())
print("person.values(): ", person.values())
print("person.items(): ", person.items())

#Schleifen
#=========================
print("Schleifen")
print("=========")

y = []
for x in range(10):
    y.append(x**2)

print(y)

y = [x**2 for x in range(10)]

plt.plot(range(10), y) #plt.plot(x-werten,y-werten)
plt.ylabel("y(x)=x²")
plt.xlabel("x")
plt.show()

words = ["Apfel", "Banane", "Kirsche", "Pfirsich", "Traube"]
longest_word = ""

for word in words:
    if len(word) > len(longest_word):
        longest_word = word

print("Das längste Wort ist:", longest_word)

#Funktionen
#=========================
print("\nFunktionen")
print("==========")

#Funktionen sind wiederverwendbare Codeblöcke die durch def definiert werden
#Funktionen können mehrere Rückgabewerte haben
def sum_and_product(x=1, y=1): #wenn parameter nicht angegeben wird, dann wird 1 angenommen (default)
    summe = x + y
    produkt = x * y
    return summe, produkt

summe, produkt = sum_and_product(3, 4)

sum_and_product(3) #x vorgegeben, y = 1
sum_and_product(y=3) #y vorgegeben, x = 1


print("Summe und Produkt ():", sum_and_product())
print("Summe und Produkt (3,4):", summe, produkt)