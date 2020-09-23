load('suturing_new_pp.mat')

figure(1)
plot3(em_data_adj(1,:,1), em_data_adj(2,:,1), em_data_adj(3,:,1), 'r-', 'LineWidth', 2)
hold on
plot3(em_data_adj(1,:,2), em_data_adj(2,:,2), em_data_adj(3,:,2), 'k-', 'LineWidth', 2)
hold on

quiver3(em_data_adj(1,1:10:end,1),em_data_adj(2,1:10:end,1),em_data_adj(3,1:10:end,1), ...
    -em_data_adj(4,1:10:end,1),-em_data_adj(5,1:10:end,1),-em_data_adj(6,1:10:end,1),'b', 'LineWidth',0.5,'MaxHeadSize',0.05);
hold on

quiver3(em_data_adj(1,1:10:end,2),em_data_adj(2,1:10:end,2),em_data_adj(3,1:10:end,2), ...
    -em_data_adj(4,1:10:end,2),-em_data_adj(5,1:10:end,2),-em_data_adj(6,1:10:end,2),'b', 'LineWidth',0.5,'MaxHeadSize',0.05);
