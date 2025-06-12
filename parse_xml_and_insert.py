import psycopg2
from lxml import etree
import datetime

# Config DB
conn = psycopg2.connect(dbname="frigo_logger", user="postgres", password="postgres", host="localhost")
cur = conn.cursor()

# Parsing XML
tree = etree.parse("continuousRead.xml")
root = tree.getroot()

# Trova ogni variabile Modbus da leggere
for elem in root.xpath("//XML-Class[@XML-Type='ModbusRTU.Template.TTemplateVariableModbus']"):
    enable = elem.get("Enable", "False")
    if enable != "True":
        continue
    try:
        nome = elem.get("Name")
        indirizzo = int(elem.get("Address"))
        # Qui simulo il valore, sostituiscilo con lettura reale da ESP32
        valore = 0  # <-- placeholder

        cur.execute("""
            INSERT INTO misure_frigo (id, registro, valore, timestamp_lettura)
            VALUES (%s, %s, %s, %s);
        """, (nome, indirizzo, valore, datetime.datetime.now()))
    except Exception as e:
        print("Errore:", e)

conn.commit()
cur.close()
conn.close()
