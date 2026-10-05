#!/usr/bin/env bash

# ============================================================
# Instalación automática de Docker Engine
# Ubuntu 22.04 LTS
# ============================================================

set -euo pipefail

# Imágenes utilizadas en las prácticas
ROBOT_IMAGE="intelligent-robotics:humble"
PYTHON_IMAGE="python:3.10"
DOCKERFILE_URL="https://raw.githubusercontent.com/elena-villalba/intelligent-robotics-student-kit/main/Dockerfile"

# Colores
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

ok() {
    echo -e "${GREEN}[OK]${NC} $1"
}

warning() {
    echo -e "${YELLOW}[AVISO]${NC} $1"
}

error() {
    echo -e "${RED}[ERROR]${NC} $1"
    exit 1
}

echo
echo "============================================================"
echo "          INSTALACIÓN AUTOMÁTICA DE DOCKER"
echo "============================================================"
echo

# ------------------------------------------------------------
# 1. Comprobar privilegios
# ------------------------------------------------------------

if [[ $EUID -eq 0 ]]; then
    # Si se ejecuta con sudo, configuramos el usuario original.
    if [[ -n "${SUDO_USER:-}" && "$SUDO_USER" != "root" ]]; then
        TARGET_USER="$SUDO_USER"
    else
        error "Ejecuta este script como usuario normal con sudo, no directamente como root."
    fi
else
    TARGET_USER="$USER"

    info "Solicitando privilegios sudo..."
    sudo -v || error "No se han podido obtener privilegios sudo."
fi

info "Usuario que será añadido al grupo docker: $TARGET_USER"

# ------------------------------------------------------------
# 2. Comprobar sistema operativo
# ------------------------------------------------------------

if [[ ! -f /etc/os-release ]]; then
    error "No se puede identificar el sistema operativo."
fi

source /etc/os-release

if [[ "${ID:-}" != "ubuntu" ]]; then
    error "Este script está diseñado exclusivamente para Ubuntu."
fi

if [[ "${VERSION_ID:-}" != "22.04" ]]; then
    error "Este script requiere Ubuntu 22.04. Detectado: ${VERSION_ID:-desconocido}"
fi

ok "Sistema operativo detectado: Ubuntu $VERSION_ID ($VERSION_CODENAME)"

# ------------------------------------------------------------
# 3. Actualizar repositorios
# ------------------------------------------------------------

info "Actualizando repositorios APT..."

sudo apt-get update

ok "Repositorios actualizados."

# ------------------------------------------------------------
# 4. Instalar dependencias
# ------------------------------------------------------------

info "Instalando ca-certificates y curl..."

sudo apt-get install -y ca-certificates curl

ok "Dependencias instaladas."

# ------------------------------------------------------------
# 5. Añadir clave GPG oficial de Docker
# ------------------------------------------------------------

info "Configurando la clave GPG oficial de Docker..."

sudo install -m 0755 -d /etc/apt/keyrings

sudo curl -fsSL \
    https://download.docker.com/linux/ubuntu/gpg \
    -o /etc/apt/keyrings/docker.asc

sudo chmod a+r /etc/apt/keyrings/docker.asc

ok "Clave GPG de Docker instalada."

# ------------------------------------------------------------
# 6. Añadir repositorio oficial Docker
# ------------------------------------------------------------

info "Añadiendo repositorio oficial de Docker..."

ARCHITECTURE="$(dpkg --print-architecture)"

sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: ${VERSION_CODENAME}
Components: stable
Architectures: ${ARCHITECTURE}
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt-get update

ok "Repositorio oficial de Docker configurado."

# ------------------------------------------------------------
# 7. Instalar Docker
# ------------------------------------------------------------

info "Instalando Docker Engine..."

sudo apt-get install -y \
    docker-ce \
    docker-ce-cli \
    containerd.io \
    docker-buildx-plugin \
    docker-compose-plugin

ok "Paquetes de Docker instalados."

# ------------------------------------------------------------
# 8. Comprobar/arrancar Docker
# ------------------------------------------------------------

info "Comprobando servicio Docker..."

if ! sudo systemctl is-active --quiet docker; then
    warning "Docker no está iniciado. Iniciando servicio..."
    sudo systemctl start docker
fi

if sudo systemctl is-active --quiet docker; then
    ok "El servicio Docker está ACTIVO."
else
    error "Docker no ha podido iniciarse."
fi

# Habilitar Docker al arrancar
sudo systemctl enable docker >/dev/null 2>&1

ok "Docker está configurado para iniciarse automáticamente."

# ------------------------------------------------------------
# 9. Mostrar versiones
# ------------------------------------------------------------

echo
info "Versiones instaladas:"

docker --version
docker compose version

# ------------------------------------------------------------
# 10. Primera prueba con sudo
# ------------------------------------------------------------

echo
info "Ejecutando prueba inicial: sudo docker run hello-world"

if sudo docker run --rm hello-world >/dev/null 2>&1; then
    ok "Docker funciona correctamente con sudo."
else
    error "La prueba 'sudo docker run hello-world' ha fallado."
fi

# ------------------------------------------------------------
# 11. Post-instalación: grupo docker
# ------------------------------------------------------------

echo
info "Configurando Docker para ejecutarlo sin sudo..."

# Docker normalmente crea este grupo durante la instalación.
# Lo creamos únicamente si no existe.
if ! getent group docker >/dev/null 2>&1; then
    sudo groupadd docker
    ok "Grupo docker creado."
else
    ok "El grupo docker ya existe."
fi

# Añadir usuario al grupo docker solamente si todavía no pertenece.
if id -nG "$TARGET_USER" | grep -qw docker; then
    ok "El usuario $TARGET_USER ya pertenece al grupo docker."
else
    sudo usermod -aG docker "$TARGET_USER"
    ok "Usuario $TARGET_USER añadido al grupo docker."
fi

# ------------------------------------------------------------
# 12. Comprobar Docker sin sudo
# ------------------------------------------------------------

echo
info "Comprobando Docker como usuario $TARGET_USER sin sudo..."

# sg permite probar el nuevo grupo sin tener que cerrar sesión
# ni ejecutar newgrp dentro del script.

if sudo -u "$TARGET_USER" sg docker -c \
    'docker run --rm hello-world >/dev/null 2>&1'; then

    ok "Docker funciona correctamente SIN sudo."

else
    error "Docker está instalado, pero la prueba sin sudo ha fallado."
fi

# ------------------------------------------------------------
# 13. Preparar imágenes Docker para Intelligent Robotics
# ------------------------------------------------------------

echo
info "Preparando las imágenes Docker utilizadas en las prácticas..."

TARGET_UID="$(id -u "$TARGET_USER")"
TARGET_GID="$(id -g "$TARGET_USER")"

BUILD_DIR="$(mktemp -d)"
chmod 755 "$BUILD_DIR"

cleanup() {
    rm -rf "$BUILD_DIR"
}
trap cleanup EXIT

info "Descargando el Dockerfile de Intelligent Robotics..."

if curl -fsSL "$DOCKERFILE_URL" -o "$BUILD_DIR/Dockerfile"; then
    ok "Dockerfile descargado correctamente."
else
    error "No se ha podido descargar el Dockerfile desde GitHub."
fi

info "Construyendo la imagen $ROBOT_IMAGE..."
info "UID/GID del usuario $TARGET_USER: $TARGET_UID/$TARGET_GID"

if sudo -u "$TARGET_USER" sg docker -c \
    "docker build --build-arg USER_UID=$TARGET_UID --build-arg USER_GID=$TARGET_GID -t $ROBOT_IMAGE '$BUILD_DIR'"; then
    ok "Imagen $ROBOT_IMAGE construida correctamente."
else
    error "No se ha podido construir la imagen $ROBOT_IMAGE."
fi

info "Descargando la imagen $PYTHON_IMAGE para el ejercicio introductorio..."

if sudo -u "$TARGET_USER" sg docker -c "docker pull $PYTHON_IMAGE"; then
    ok "Imagen $PYTHON_IMAGE descargada correctamente."
else
    error "No se ha podido descargar la imagen $PYTHON_IMAGE."
fi

# ------------------------------------------------------------
# 14. Comprobaciones finales
# ------------------------------------------------------------

echo
echo "============================================================"
echo "                  COMPROBACIÓN FINAL"
echo "============================================================"
echo

ERRORS=0

if command -v docker >/dev/null 2>&1; then
    ok "Comando docker instalado."
else
    echo -e "${RED}[ERROR]${NC} Comando docker no encontrado."
    ERRORS=$((ERRORS + 1))
fi

if sudo systemctl is-active --quiet docker; then
    ok "Servicio docker activo."
else
    echo -e "${RED}[ERROR]${NC} Servicio docker inactivo."
    ERRORS=$((ERRORS + 1))
fi

if docker compose version >/dev/null 2>&1; then
    ok "Docker Compose instalado."
else
    echo -e "${RED}[ERROR]${NC} Docker Compose no disponible."
    ERRORS=$((ERRORS + 1))
fi

if sudo -u "$TARGET_USER" sg docker -c \
    'docker info >/dev/null 2>&1'; then
    ok "Usuario $TARGET_USER puede utilizar Docker sin sudo."
else
    echo -e "${RED}[ERROR]${NC} El usuario no puede acceder a Docker sin sudo."
    ERRORS=$((ERRORS + 1))
fi

if sudo -u "$TARGET_USER" sg docker -c \
    "docker image inspect $ROBOT_IMAGE >/dev/null 2>&1"; then
    ok "Imagen $ROBOT_IMAGE disponible."
else
    echo -e "${RED}[ERROR]${NC} Imagen $ROBOT_IMAGE no disponible."
    ERRORS=$((ERRORS + 1))
fi

if sudo -u "$TARGET_USER" sg docker -c \
    "docker image inspect $PYTHON_IMAGE >/dev/null 2>&1"; then
    ok "Imagen $PYTHON_IMAGE disponible."
else
    echo -e "${RED}[ERROR]${NC} Imagen $PYTHON_IMAGE no disponible."
    ERRORS=$((ERRORS + 1))
fi

echo

if [[ "$ERRORS" -eq 0 ]]; then
    echo -e "${GREEN}============================================================${NC}"
    echo -e "${GREEN}   ✓ DOCKER INSTALADO Y CONFIGURADO CORRECTAMENTE${NC}"
    echo -e "${GREEN}============================================================${NC}"
    echo
    echo "Usuario:        $TARGET_USER"
    echo "Docker:         $(docker --version)"
    echo "Docker Compose: $(docker compose version --short)"
    echo "Robot image:    $ROBOT_IMAGE"
    echo "Python image:   $PYTHON_IMAGE"
    echo
    echo -e "${YELLOW}IMPORTANTE:${NC}"
    echo "Cierra sesión y vuelve a entrar para que el grupo"
    echo "'docker' se aplique a tu sesión actual."
    echo
    echo "Después podrás ejecutar:"
    echo
    echo "    docker run hello-world"
    echo
else
    echo -e "${RED}Se han encontrado $ERRORS errores durante la comprobación.${NC}"
    exit 1
fi
