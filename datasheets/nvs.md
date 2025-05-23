# Non-Volatil Storage

- **Calibração do giroscópio**: Calcula-se o bias que é subtraído dos dados lidos. Produz `Vec3f gyroData = Vec3f(0.0f, 0.0f, 0.0f);`

## Algoritmo para uso do NVS com flag de bias do giroscópio

1. Inicializar o NVS.
2. Verificar se existe a flag indicando bias salvo:
    - Se sim, ler o bias salvo do NVS.
    - Se não, calcular o bias, salvar no NVS e setar a flag.
3. Subtrair o bias dos dados lidos do giroscópio.
4. Utilizar os dados corrigidos para o processamento.


