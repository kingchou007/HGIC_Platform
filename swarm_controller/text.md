好的,根据你提供的论文,这里是我对你的数学表达式方法的一些优化建议:

1. 使用向量符号表示位置和速度,如$\vec{p}_i$,$\vec{v}_i$,以更加凸显它们是向量。
2. 对势函数用 $U$ 表示,如吸引势$U_\text{att}$,排斥势$U_\text{rep}$,统一表示。
3. 使用指数函数表达排斥势,使其衰减更平滑,如:

$$
U_\text{rep} = \begin{cases}
A\exp(-r/B), & r < r_\text{rep} \
0, & r \geq r_\text{rep}
\end{cases}
$$

4. 让吸引势也与距离相关,近处效果更强:

$$
U_\text{att} = C/r
$$

5. 总势能为各势能叠加,用$\nabla$表示势能的梯度力:

$$
\vec{F} = \nabla U_\text{rep} + \nabla U_\text{att}
$$

6. 在速度更新中考虑平滑因子$\alpha$,如 $v_{t+1} = \alpha v_t + (1-\alpha)v^*$
7. 用碰撞避免算法生成约束,代入优化求解无人机的最优轨迹
8. 明确定义各变量和参数的物理意义
