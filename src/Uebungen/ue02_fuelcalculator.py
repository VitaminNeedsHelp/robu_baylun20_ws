class FuelCalculator:
    def __init__(self, driven_km, consumption_in_l): #Konstructor
        self._driven_km = driven_km                   #Attribut
        self._consumption_in_l = consumption_in_l

        if driven_km <= 0.0:
            raise ValueError("Die gefahrenen Kilometer müssen größer als 0 sein.")
        
        if consumption_in_l <= 0.0:
            raise ValueError("Der Verbrauch muss größer als 0 sein.")
        
        self._calc()

    def _calc(self):
        self._avg_consumption_l_100km = self._consumption_in_l / self._driven_km * 100.0

    def get_avg_consumption_l(self):
        return self._avg_consumption_l_100km
    
    def __str__(self):
        return f"{'Durchschnittsverbrauch:':25s}{self._avg_consumption_l_100km:.2f} l/100km"


class FuelUI:
    def __init__(self):
        try:
            self.input()
            self.calc()
            self.output()
        except Exception as e:
            print(e)
    
    def input(self):
        print("Fuel Calculator")
        print("===============")

        self._driven_km = float(input(f"{'Kilometer driven:':25s}"))
        self._consumption_in_l = float(input(f"{'Consumption in l:':25s}"))

    def calc(self):
        self._fuelcalculator = FuelCalculator(
            self._driven_km,
            self._consumption_in_l)

    def output(self):
        print(self._fuelcalculator)

if __name__ == "__main__": #Started das program nur wenn es direkt gestartet wird und nicht importiert wird
    FuelUI()

# fuelcalculator = FuelCalculator(1000, 60)

# print(f"Der durchschnittliche Verbrauch beträgt {fuelcalculator.get_avg_consumption_l():.2f} l/100km")
# print(fuelcalculator)