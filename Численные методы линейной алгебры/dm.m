clear
clc

%% Исходные данные
% матрица А
A=[ 2.0000    1.1100    0.6667    0.5000    0.4000 ;
    1.1100    0.6667    0.5000    0.4000    0.3333 ;
    0.6667    0.5000    0.4000    0.3333    0.2857 ;
    0.5000    0.4000    0.3333    0.2857    0.2500 ;
    0.4000    0.3333    0.2857    0.2500    0.2222 ]

% вектор правой части b
b=[ 3.1167
    2.0333
    1.5429
    1.2512
    1.0552  ]


%% Метод Гаусса

 %{
AB=[A b]             %% Матрица расширения
n = length(b)        %% Размерность матрицы

for j =1:n-1                      %% прямой преобразование       U
    for i =j+1:n
    k = AB(i,j)/AB(j,j)
    AB(i,:)=AB(i,:)-k*AB(j,:)
    end
end
AB

for y =1:n-1                     %% обратное преобразование      D
    z = n-y+1
    for x=1:n-y
        k = AB(x,z)/AB(z,z)
        AB(x,:)=AB(x,:)-k*AB(z,:)
    end
end
AB

for l=1:n                          %% 3-ое преобразование      I
    AB(l,:) = AB(l,:)/AB(l,l)
end
AB

x = AB(:,6)
q = A*x-b
delta = norm(q)

%% Bозмущенной системы
A_disturbed = A
A_disturbed(4, 1) = A(4, 1) + 0.01
A_disturbed(1, 4) = A(1, 4) + 0.01

Ab=[A_disturbed b]             %% Матрица расширения
n = length(b)        %% Размерность матрицы

for j =1:n-1                      %% прямой преобразование       U
    for i =j+1:n
    k = Ab(i,j)/Ab(j,j)
    Ab(i,:)=Ab(i,:)-k*Ab(j,:)
    end
end
Ab

for y =1:n-1                     %% обратное преобразование      D
    z = n-y+1
    for x=1:n-y
        k = Ab(x,z)/Ab(z,z)
        Ab(x,:)=Ab(x,:)-k*Ab(z,:)
    end
end
Ab

for l=1:n                          %% 3-ое преобразование      I
    Ab(l,:) = Ab(l,:)/Ab(l,l)
end
Ab

x_disturbed = Ab(:,6)
q_disturbed = A_disturbed*x_disturbed-b
delta_disturbed = norm(q_disturbed)

%% Проверяем решение невозмущенного и возмущенного
delta_relative = abs(delta_disturbed-delta)/abs(delta)

%% наибольшее и наименьшее собственные значения
eig(A)
max(eig(A))
min(eig(A))

eig(A_disturbed)
max(eig(A_disturbed))
min(eig(A_disturbed))

%% собственные векторы
[V,D] = eig(A)
[V_disturbed,D_disturbed] = eig(A_disturbed)
V
V_disturbed
%}


%%  QR-разложение   Gram-Schmidt process
%{
% Векторы-столбцы матрицы A     Инициализация
A1 = A(:,1)
A2 = A(:,2)
A3 = A(:,3)
A4 = A(:,4)
A5 = A(:,5)

% Проекция и вычитание
Q1 = A1

C1 = sum(A2.*Q1)/sum(Q1.*Q1)
Q2 = A2-C1*Q1

C1 = sum(A3.*Q1)/sum(Q1.*Q1)
C2 = sum(A3.*Q2)/sum(Q2.*Q2)
Q3 = A3-C1*Q1-C2*Q2

C1 = sum(A4.*Q1)/sum(Q1.*Q1)
C2 = sum(A4.*Q2)/sum(Q2.*Q2)
C3 = sum(A4.*Q3)/sum(Q3.*Q3)
Q4 = A4-C1*Q1-C2*Q2-C3*Q3

C1 = sum(A5.*Q1)/sum(Q1.*Q1)
C2 = sum(A5.*Q2)/sum(Q2.*Q2)
C3 = sum(A5.*Q3)/sum(Q3.*Q3)
C4 = sum(A5.*Q4)/sum(Q4.*Q4)
Q5 = A5-C1*Q1-C2*Q2-C3*Q3-C4*Q4

% Q матрицa    Нормализация
Q(:,1) = Q1/norm(Q1)
Q(:,2) = Q2/norm(Q2)
Q(:,3) = Q3/norm(Q3)
Q(:,4) = Q4/norm(Q4)
Q(:,5) = Q5/norm(Q5)

R = Q'*A
y = Q'*b
x = R\y
A*x-b
delta = norm(A*x-b)

% Для возмущенной системой:
A_disturbed = A
A_disturbed(4, 1) = A(4, 1) + 0.01
A_disturbed(1, 4) = A(1, 4) + 0.01

A1 = A_disturbed(:,1)
A2 = A_disturbed(:,2)
A3 = A_disturbed(:,3)
A4 = A_disturbed(:,4)
A5 = A_disturbed(:,5)

% Проекция и вычитание
Q1 = A1

C1 = sum(A2.*Q1)/sum(Q1.*Q1)
Q2 = A2-C1*Q1

C1 = sum(A3.*Q1)/sum(Q1.*Q1)
C2 = sum(A3.*Q2)/sum(Q2.*Q2)
Q3 = A3-C1*Q1-C2*Q2

C1 = sum(A4.*Q1)/sum(Q1.*Q1)
C2 = sum(A4.*Q2)/sum(Q2.*Q2)
C3 = sum(A4.*Q3)/sum(Q3.*Q3)
Q4 = A4-C1*Q1-C2*Q2-C3*Q3

C1 = sum(A5.*Q1)/sum(Q1.*Q1)
C2 = sum(A5.*Q2)/sum(Q2.*Q2)
C3 = sum(A5.*Q3)/sum(Q3.*Q3)
C4 = sum(A5.*Q4)/sum(Q4.*Q4)
Q5 = A5-C1*Q1-C2*Q2-C3*Q3-C4*Q4

% Q матрицa    Нормализация
Q_disturbed(:,1) = Q1/norm(Q1)
Q_disturbed(:,2) = Q2/norm(Q2)
Q_disturbed(:,3) = Q3/norm(Q3)
Q_disturbed(:,4) = Q4/norm(Q4)
Q_disturbed(:,5) = Q5/norm(Q5)

R_disturbed = Q_disturbed'*A_disturbed
y_disturbed = Q_disturbed'*b
x_disturbed = R_disturbed\y
A_disturbed*x_disturbed-b
delta_disturbed= norm(A_disturbed*x_disturbed-b)

delta_relative = norm(x_disturbed-x)/norm(x)

%}
