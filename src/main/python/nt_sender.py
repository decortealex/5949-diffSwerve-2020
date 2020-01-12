from networktables import NetworkTables as nt

nt.initialize(server='10.59.49.2')

def send_data(table_name, name, value):
    table = nt.getTable(table_name)
    table.putNumber(name, value)
