# ==========================================
# MAKEFILE PURO PARA MSP430 GCC
# ==========================================

# 1. Herramientas y Microcontrolador
CC = msp430-elf-gcc
MCU = msp430f5529

# 2. Rutas del Compilador de Texas Instruments
# Aquí es donde TI guarda el archivo msp430.h y los mapas de memoria (.ld)
TI_INCLUDE = /opt/ti/include

# 3. Flags de Compilación
# -O0 para desarrollo, -Wall para avisos, -g para info de debug
CFLAGS = -mmcu=$(MCU) -I$(TI_INCLUDE) -O0 -Wall -Wpedantic -Wextra -Wconversion -Wsign-conversion -Wdouble-promotion -g
LDFLAGS = -L$(TI_INCLUDE)

# 4. Buscador automático de archivos (Magia pura)
SOURCES = $(wildcard *.c)
OBJECTS = $(SOURCES:.c=.o)
TARGET = mi_robot.elf

# ==========================================
# COMANDOS
# ==========================================

# Comando por defecto: compilar todo
build: $(TARGET)

# Cómo juntar todos los pedazos en el archivo final (.elf)
$(TARGET): $(OBJECTS)
	@echo "=> Enlazando el programa final: $(TARGET)"
	$(CC) $(CFLAGS) $(LDFLAGS) -o $(TARGET) $(OBJECTS)

# Cómo compilar cada archivo .c individualmente
%.o: %.c
	@echo "=> Compilando $<..."
	$(CC) $(CFLAGS) -c $< -o $@

# Comando para subir el código al robot
flash: build
	@echo "=> Flasheando el microcontrolador..."
	mspdebug tilib "prog $(TARGET)"

# Comando para limpiar la basura compilada
clean:
	@echo "=> Borrando archivos compilados..."
	rm -f $(OBJECTS) $(TARGET)

# Comando maestro (alias para hacerlo todo de golpe)
run: flash
