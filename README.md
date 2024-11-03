# Quest02
Repositório contendo arquivos da segunda questão da Prova Prática do Processo de Seleção para Pesquisador I do SENAI. Candidato: Danilo Giacomin Schneider

# Conteúdo

- pacote_1 publica mensagens a cada 1 (um) segundo com informações sobre a quantidade total de memória, o uso de memória RAM em Gigabyte e o percentual do uso;

-pacote_2 simula a leitura de um sensor com uma taxa de amostragem de 1 Hz. Os dados do sensor passam por um filtro de média móvel considerando os últimos 5 valores adquiridos pelo sensor. Esse pacote provem duas interfaces de serviço, a primeira retorna os últimos 64 resultados gerados pelo filtro, e a segunda zera os dados gerados pelo filtro;

-pacote_3 encontra, via requisição de ação, o décimo número primo, gerando respostas intermediárias pela interface de ação e também o resultado final.

# Instalação

Use the next commands to install docker engine and docker compose (instructions from official website):

```
# uninstall conflicting packages
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# post-installation steps

sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

# verify installation
docker run hello-world

```

Clone the repository and build the image/run the container

```
mkdir danilo
cd danilo
git clone https://github.com/danilogsch/Quest02.git
cd Quest02
docker compose up -d dev
docker compose exec -it dev bash

```