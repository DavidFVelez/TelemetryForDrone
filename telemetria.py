import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
import serial
import threading

# Configuración del puerto serie y el baudrate
port = 'COM4'
baudrate = 115200

# Creamos listas vacías para almacenar el valor de cada sensor en sus respectivas coordenadas
aX, aY, aZ, gX, gY, gZ, mX, mY, mZ = [], [], [], [], [], [], [], [], []

# Se obtienen los datos por puerto serial
def puertoSerial():
    with serial.Serial(port, baudrate) as pSerial:
        while True:
            # Se separa por comas la información que llega por puerto serial
            datos = pSerial.readline().decode('utf-8').strip().split(',')
            
            #Se asigna el valor de cada medida.
            if len(datos) == 13:
                aX.append(float(datos[0]))
                aY.append(float(datos[1]))
                aZ.append(float(datos[2]))
                gX.append(float(datos[3]))
                gY.append(float(datos[4]))
                gZ.append(float(datos[5]))
                mX.append(float(datos[6]))
                mY.append(float(datos[7]))
                mZ.append(float(datos[8]))

                # Se imprimen los datos por consola
                print(f'Aceleracion: {datos[0]}, {datos[1]}, {datos[2]}')
                print(f'Giroscopio: {datos[3]}, {datos[4]}, {datos[5]}')
                print(f'Magnetometro: {datos[6]}, {datos[7]}, {datos[8]}')


# Se crea a figura para las graficas
fig, eje = plt.subplots(figsize=(10, 10))
fig.subplots_adjust(hspace=0.5, wspace=0.3)

# Se crean estados de graficar para poder mostrar o ucultar cada medida
estados = {
    'a_X': True,
    'a_Y': True,
    'a_Z': True,
    'g_X': True,
    'g_Y': True,
    'g_Z': True,
    'm_X': True,
    'm_Y': True,
    'm_Z': True,
}

# Se activa o desactiva respectivamente el grafico de cada variable
def activacion(event, variable):
    estados[variable] = not estados[variable]

# Se actualizan las gráficas sincronicamente
def actualizar(frame):
    eje.clear()

    # Si botón se activo entonces se muestra la respectiva gráfica de lo contrario se oculta
    if estados['a_X']:
        eje.plot(range(len(aX)), aX, label='aX')
    if estados['a_Y']:
        eje.plot(range(len(aY)), aY, label='aY')
    if estados['a_Z']:
        eje.plot(range(len(aZ)), aZ, label='aZ')
    if estados['g_Y']:
        eje.plot(range(len(gX)), gX, label='gX')
    if estados['g_Y']:
        eje.plot(range(len(gY)), gY, label='gY')
    if estados['g_Z']:
        eje.plot(range(len(gZ)), gZ, label='gZ')
    if estados['m_X']:
        eje.plot(range(len(mX)), mX, label='mX')
    if estados['m_Y']:
        eje.plot(range(len(mY)), mY, label='mY')
    if estados['m_Z']:
        eje.plot(range(len(mZ)), mZ, label='mZ')
    eje.legend(loc='upper left')
# Se implementan botones para mostrar las graficas
botones = []
for i, variable in enumerate(estados.keys()):
    boton = Button(plt.axes([0.9, 0.7 - i * 0.05, 0.1, 0.05]), variable, color='lightblue', hovercolor='lightgrey')
    boton.on_clicked(lambda event, name=variable: activacion(event, name))
    botones.append(boton)

# Se crea la ventana animada y con actulización de graficas
animacion = FuncAnimation(fig, actualizar, interval=100)

# Contenedor de datos en segundo plano
thread = threading.Thread(target=puertoSerial)
thread.start()

# Se muestran las gráficas
plt.show()

