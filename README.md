# Control Arm Twink

Digital twin de um braço RRR de 3 GDL: interface web em three.js,
planejamento de movimento com MoveIt 2 e um robô físico controlado por
ESP32.

## Arquitetura

O MoveIt roda **só como serviço de planejamento**. Quem executa a
trajetória é a interface (gêmeo virtual) e, quando calibrado, o loop
PID do backend (robô real).

```
frontend/urdf.html ──fetch──► backend/main.py ──HTTP──► moveit_bridge ──► move_group
   three.js                     FastAPI                  (container)        (OMPL + KDL)
      │                            │
      │                            └──serial──► ESP32 ──► motores DC + potenciômetros
      └── reproduz a trajetória planejada
```

O `moveit_bridge` também publica `/joint_states`, então o RViz dentro do
container mostra exatamente a mesma pose do braço do three.js.

## Subir

O ROS e o MoveIt vivem inteiramente no container — não é preciso
instalar ROS na máquina (funciona igual em Windows e Linux).

```bash
docker compose up -d --build
```

Isso sobe o `move_group` e a ponte HTTP em `localhost:8081`.
Para ver a cena no RViz, troque `RVIZ` para `"true"` no
`docker-compose.yml` e abra `http://localhost:6080/vnc.html`.

Depois, o backend:

```bash
pip install -r requirements.txt
```

```bash
cd backend && uvicorn main:app --reload
```

A interface fica em `http://localhost:8000/urdf`.

Se o backend rodar em outra máquina que não a do container, aponte
`MOVEIT_URL` para o endereço certo (o padrão é `http://localhost:8081`).

## Fluxo na interface

1. **Alvo** — arraste a esfera rosa (TCP) no 3D, ou use os sliders.
2. **Planejar** — o MoveIt calcula a trajetória e um robô fantasma azul
   a repete em loop. Nada se move de verdade.
3. **Executar** — o braço reproduz a trajetória planejada.

O seletor **posição / limites** no topo do painel troca o que os sliders
editam. Em *limites*, cada junta ganha dois sliders (mín em âmbar, máx
em teal) e a área de trabalho é refeita a cada arrasto. `↺ URDF`
devolve os limites do modelo.

Todas as três juntas cobrem os 360°, mas a janela entre mínimo e máximo
nunca passa de **180°**: ao alargá-la, o outro extremo é empurrado junto
e ela desliza em vez de crescer.

O volume desenhado é a região alcançável de verdade, extraída por
marching squares sobre as amostras de cinemática direta. Preencher um
intervalo por fatia de altura seria uma caixa envolvente, e mediu 28%
de área que o braço não alcança.

Restringir `revbase` deixa o volume de ser uma revolução completa. Como
todas as juntas depois dela giram em torno de y, a componente lateral do
TCP é constante (−28,5 mm) e o braço varre uma região plana, girada em
torno de z. Essa região cruza o eixo, então alcançar para frente e para
trás são dois azimutes separados por ~180°.

Os limites operacionais são sempre um subconjunto dos do URDF e vão
junto no pedido de planejamento como restrição de caminho, senão o
MoveIt planejaria com os limites do URDF e a trajetória sairia da faixa
escolhida — prender só o alvo não basta.

Dois botões de visualização:

- **Área de trabalho** — o volume alcançável ao redor do braço. Não é
  maciço: o braço não alcança o próprio miolo, porque a distância
  mínima do ombro ao TCP é 149 mm, e esse vazio aparece recortado.
- **Trajetória** — o caminho do end-effector (`tool0`) do último plano.

## Poses

Escolha um alvo (sliders ou arrastando o TCP) e **Salvar pose**. Cada
pose salva pode ser executada sozinha pelo `▶`, ou todas em ordem por
**Executar sequência**. A lista persiste no navegador.

O que se guarda é a **configuração de juntas**, não a posição do TCP:
uma mesma posição tem mais de uma solução, e a que interessa é a que o
braço estava assumindo. Uma pose salva sob outros limites pode cair
fora da janela atual — nesse caso ela aparece marcada, seu `▶` fica
desabilitado e a sequência a pula.

O modelo do volume existe mesmo com a visualização desligada, porque é
ele que impede arrastar o TCP para fora do alcance: o alvo é travado na
amostra alcançável mais próxima. Um ponto que já está dentro passa
intacto, então não se perde precisão onde se trabalha.

O envio ao robô real é um checkbox separado, **desligado por padrão**:
a conversão ADC↔radiano ainda é provisória (ver `adc_to_degree` em
`backend/main.py`), então uma trajetória planejada ainda não corresponde
a ângulos corretos no braço físico.

## Notas do modelo

- O URDF foi reexportado do Fusion com **PLA** (≈1250 kg/m³) no lugar do
  aço: massas e inércias foram reescaladas por 1250/7850.
- Com 3 GDL só dá para comandar **posição** do TCP — a orientação é
  consequência. Daí `position_only_ik: true` em `kinematics.yaml`.
- O frame do link `gripper_1` fica a 370 mm da garra real (origem de
  junta estranha vinda do exportador), o que o torna inútil como alvo de
  IK. O link `tool0` corrige isso: é fixo, sem massa, posicionado na
  ponta real medida do STL, e é ele a ponta da cadeia no SRDF.
- `revbase` é uma junta contínua, então o planejador pode devolver
  ângulos fora de ±180°; a interface normaliza antes de exibir e antes
  de converter para o robô real.

## Pendências

- **Calibração ADC↔radiano** por junta (ganho, offset e sinal). É o que
  falta para o robô real seguir uma trajetória do MoveIt.
- Limites de velocidade/aceleração em `joint_limits.yaml` são
  conservadores e chutados; devem ser medidos no braço real.
- Migrar para `ros2_control` + `joint_trajectory_controller` quando fizer
  sentido — `config/moveit_controllers.yaml` já declara o
  `arm_controller` previsto.

## Pacotes

- `problemas2_urdf_description/` — URDF/xacro, malhas STL, launches
- `problemas2_moveit_config/` — SRDF, cinemática, limites, launch do
  `move_group`
- `moveit_bridge/` — nó rclpy que expõe o MoveIt por HTTP
- `backend/` — FastAPI, loop PID 100 Hz, serial do ESP32
- `frontend/` — dashboard e viewer 3D
