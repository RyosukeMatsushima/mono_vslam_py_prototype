import MySQLdb

connection = MySQLdb.connect(
    host='db',
    port=3306,
    user='docker',
    passwd='docker',
    db='mono_vslam_sample'
    )
cursor = connection.cursor()

connection.commit()

connection.close()
