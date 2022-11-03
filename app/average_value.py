
class AverageValue:

    def __init__(self, average_range):
        self.average_range = average_range
        self.values = []

        self.average = 0.0
        self.variance = 0.0

    def update(self, value):

        self.values.append( value )
        if len(self.values) > self.average_range:
            self.values.pop(0)

        self.average = sum(self.values) / len(self.values)
        self.variance = sum( [ abs( value - self.average ) for value in self.values ] ) / len(self.values)

    def value(self):
        return self.average

    def reset(self):
        self.values = []

