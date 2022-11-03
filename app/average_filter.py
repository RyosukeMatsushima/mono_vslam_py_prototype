
class AverageFilter:

    def __init__(self, average_range):
        self.average_range = average_range
        self.values = []

    def update(self, value):

        self.values.append( value )
        if len(self.values) > self.average_range:
            self.values.pop(0)

        return sum(self.values) / len(self.values)

    def reset(self):
        self.values = []

