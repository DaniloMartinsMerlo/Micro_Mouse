# Projeto MicroMouse Danilo Martins

Vídeos: 
[Explicação do código](https://youtu.be/6hLKYcfLsgU)
[Código rodando](/videos/execução.mp4)

## 1. Construindo o Workspace

```bash
colcon build
```

```bash
source install/setup.bash
```

## 2. Executando o Jogo

Para iniciar o jogo, execute o seguinte comando em um terminal:

```bash
ros2 run cg maze
```

## 3. Executando código da parte 1:

Para executar o código de navegação para identificar a rota mais otimizada você deve estar com o labirinto aberto utilizando o comando anterior e em seguida executar o seguinte comando:

```bash
ros2 run cg_move_client astar
```

## 4. Executando código da parte 2:

Para executar o mapeamento e a identificação do melhor caminho sem ter acesso ao mapa atual você deve estar com o mapa sendo executado (Iten 2) e executar o seguinte comando:

```bash
ros2 run cg_move_client move_client
```