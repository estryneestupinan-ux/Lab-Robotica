figure; hold on; grid on; axis equal;

paso1 = deg2rad(2);     % resolucion barrido eslabon 1
paso2 = deg2rad(2);     % resolucion barrido eslabon 2

XX = [];  YY = [];   % guardo puntos alcanzables

for t1 = 0:paso1:2*pi              % primer eslabon: libre
    for t2 = -2:paso2:2            % segundo eslabon: limitado a +-2 rad
        % angulos geometricos (deshago mis convenciones)
        ang1 = -t1;                % t1 positivo = izquierda
        ang2 =  t2;                % t2 positivo = derecha
        a1 = ang1;
        a2 = ang1 + ang2;
        % posicion del efector
        Xef = L1*cos(a1) + L2*cos(a2);   % X vertical
        Yef = L1*sin(a1) + L2*sin(a2);   % Y horizontal
        XX(end+1) = Xef;
        YY(end+1) = Yef;
    end
end

% grafico los puntos alcanzables (Y horizontal, X vertical)
plot(YY, XX, '.', 'Color',[0.3 0.6 1], 'MarkerSize',3);
plot(0,0,'ks','MarkerSize',12,'MarkerFaceColor','k');
xlabel('Y [cm] (horizontal)'); ylabel('X [cm] (vertical)');
title('Rango del robot (espacio de trabajo)');
lim = L1+L2+5; xlim([-lim lim]); ylim([-lim lim]);