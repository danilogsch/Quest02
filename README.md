# Quest02
Repositório contendo arquivos da segunda questão da Prova Prática do Processo de Seleção para Pesquisador I do SENAI. Candidato: Danilo Giacomin Schneider

# Conteúdo

- package_1 publica mensagens a cada 1 (um) segundo com informações sobre a quantidade total de memória, o uso de memória RAM em Gigabyte e o percentual do uso;

- package_2 simula a leitura de um sensor com uma taxa de amostragem de 1 Hz. Os dados do sensor passam por um filtro de média móvel considerando os últimos 5 valores adquiridos pelo sensor. Esse pacote provem duas interfaces de serviço, a primeira retorna os últimos 64 resultados gerados pelo filtro, e a segunda zera os dados gerados pelo filtro;

- package_3 encontra, via requisição de ação, o décimo número primo, gerando respostas intermediárias pela interface de ação e também o resultado final.

# Instalação (Testado no Ubuntu Jammy 22.04)

Use os próximos comandos para installar o docker engine e docker compose (instructions from official [website](https://docs.docker.com/desktop/install/linux/)).

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

Clone o repositório, contrua a imagem, rode o container, construa os pacotes:

```
mkdir danilo
cd danilo
git clone https://github.com/danilogsch/Quest02.git
cd Quest02
docker compose up -d dev
docker compose exec -it dev bash
colcon build
. install/setup.bash
```

Obs.: o comando "docker compose exec -it dev bash" pode ser usado diversar vezes na pasta do repositório para gerar multiplos terminais iterativos no container.

# Rodando o package_1

Dentro do container:

```
ros2 run package_1 memory_monitor_publisher
# Em um terminal diferente (também no container):
ros2 topic echo /memory_info
# Verifique a frequência de mensagens no tópico:
ros2 topic hz /memory_info
```

Um parâmetro booleano foi implementado, chamado "use_single_topic", ele controla se o nó publica todas as informações requeridas em um tópico do tipo string "/memory_info". O valor default do parâmetro é true, quando configurado em false, o nó publica as informações requeridas em 3 tópicos distintos do tipo float ("total_memory", "used_memory" e "memory_percent"). Como se trata de um parâmetro e não um argumento, ele pode ser modificado enquanto o nó roda:

```
ros2 run package_1 memory_monitor_publisher
# Em um terminal diferente (também no container):
. install/setup.bash
ros2 param set /memory_monitor_publisher use_single_topic true
# Verifique os tópicos publicados:
ros2 topic list
# Verifique a frequência de mensagens em cada tópico, e.g.:
ros2 topic hz /memory_percent
# Verifique o conteúdo das mensagens em cada tópico, e.g.:
ros2 topic echo /total_memory
```

o nó também pode ser rodado com o parâmetro false pelo comando:

```
ros2 run package_1 memory_monitor_publisher --ros-args -p "use_single_topic:=false"
```

# Rodando o package_2
Dentro do container:

```
. install/setup.bash
ros2 run package_2 sensor_node
# Em um terminal diferente (também no container):
. install/setup.bash
# Chame o serviço para retornar os últimos 64 resultados gerados pelo filtro através do código:
ros2 service call /get_filtered_data package_2/srv/GetFilteredData
# Chame o serviço para zerar os dados gerados pelo filtro:
ros2 service call /get_filtered_data package_2/srv/GetFilteredData
```

# Rodando o package_3
Dentro do container:

```
. install/setup.bash
ros2 run package_3 prime_action_server
# Em outro terminal (também no container):
ros2 action send_goal /find_prime package_3/action/FindPrime "{target_prime: 10}"
# Mude o valor de "target_prime" para "n", assim o servidor procura pelo n-ésimo número primo.
#Como os valores dos feedbacks intermediários não podem ser vistos diretamente pelo comando acima, um nó cliente foi criado para imprimir no terminal todas as informações, e pode ser rodado pelo comando:
ros2 run package_3 prime_action_client 10
# Onde o primeiro argumeto representa o "target_prime" enviado ao servidor.

```