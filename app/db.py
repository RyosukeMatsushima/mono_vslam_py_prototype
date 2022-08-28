import MySQLdb

class database:
    def __init__(self):
        self.connection = MySQLdb.connect(
            host='db',
            port=3306,
            user='docker',
            passwd='docker',
            db='mono_vslam_sample'
            )
        self.connection.autocommit(False)
        self.cursor = self.connection.cursor()

        sql = "CREATE TABLE IF NOT EXISTS key_point_positions(id INT, x INT, y INT, z INT)"
        self.cursor.execute(sql)

    def close(self):
        self.connection.commit()
        self.connection.close()

    def create(self, id, x, y, z):
        sql = "INSERT INTO key_point_positions (id, x, y, z) VALUES ({}, {}, {}, {})".format(id, x, y, z)
        self.cursor.execute(sql)
        self.connection.commit()
    
    def find(self, id):
        sql = "SELECT * FROM key_point_positions WHERE id={}".format(id)
        self.cursor.execute(sql)
        return self.cursor.fetchall()[0]
