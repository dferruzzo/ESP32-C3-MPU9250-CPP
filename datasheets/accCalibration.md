# Algoritmo de Calibração do Acelerômetro

Ref: Design and Implementation of a Real Time Attitude Estiamtion System with Low Cost Sensors. DaSilva2020.

1. Coletar ($6N$) amostras de dados $(x_i, y_i, z_i)$ do acelerômetro em repouso com:

    - O eixo $x$ apontando para cima ($N$ amostras).     
    - O eixo $x$ apontando para baixo ($N$ amostras).
    - O eixo $y$ apontando para cima ($N$ amostras).
    - O eixo $y$ apontando para baixo ($N$ amostras).
    - O eixo $z$ apontando para cima ($N$ amostras).
    - O eixo $z$ apontando para baixo ($N$ amostras).
    


2. Montar a matriz $W\in\mathbb{R}^{(6N \times 4)}$ com os dados coletados: 
$$
        W = \begin{bmatrix}
            x_1& y_1& z_1& 1\\
            x_2& y_2& z_2& 1\\
            \vdots&\vdots&\vdots&\vdots\\
            x_{6N}& y_{6N}& z_{6N}& 1
        \end{bmatrix}
$$

3. Montar a matriz $Y\in\mathbb{R}^{(6N \times 3)}$ com os dados esperados: 

$$
        Y = \begin{bmatrix}
                1& 0& 0\\
                \vdots&\vdots&\vdots\\
                1& 0& 0\\
                -1& 0& 0\\ 
                \vdots&\vdots&\vdots\\
                -1& 0& 0\\
                0& 1& 0\\
                \vdots&\vdots&\vdots\\
                0& 1& 0\\ 
                0& -1& 0\\
                \vdots&\vdots&\vdots\\
                0& -1&  0\\
                0&  0&  1\\
                \vdots&\vdots&\vdots\\
                0&  0&  1\\     
                0&  0& -1\\
                \vdots&\vdots&\vdots\\
                0&  0& -1
             \end{bmatrix}
$$

4. Computar a matriz $W_b\in\mathbb{R}^{4\times 3}$

    $$W_b = (W^T W)^{-1} W^T Y$$

5. Calcular a matriz de calibração:

$$
\begin{bmatrix} M^T\\a_{offset}^T\end{bmatrix}=W_b
$$

6. As medições calibradas $a_c\in\mathbb{R}^{3\times1}$ são obtidas:

$$
a_c^T = \begin{bmatrix} a_m^T & 1\end{bmatrix}\begin{bmatrix}M^T\\a_{offset}^T\end{bmatrix}=\begin{bmatrix} a_m^T & 1\end{bmatrix}W_b.
$$