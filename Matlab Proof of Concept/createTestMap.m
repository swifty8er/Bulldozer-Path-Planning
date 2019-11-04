%creates a proof of concept test map

function Map = createTestMap(Map)
    ini = [0,0];
    Map.Obstacles{1} = [ini(1) + 1, ini(2) + 2; ...
                        ini(1) + 4, ini(2) + 2; ...
                        ini(1) + 4, ini(2) + 5; ...
                        ini(1) + 1, ini(2) + 5; ...
                        ini(1) + 1, ini(2) + 2];
    Map.Obstacles{2} = [ini(1) + 5, ini(2) + 2; ...
                        ini(1) + 5, ini(2) + 4; ...
                        ini(1) + 6, ini(2) + 4; ...
                        ini(1) + 6, ini(2) + 2; ...
                        ini(1) + 5, ini(2) + 2];
    Map.Obstacles{3} = [ini(1) + 5, ini(2) + 0; ...
                        ini(1) + 5, ini(2) + 1; ...
                        ini(1) + 7, ini(2) + 1; ...
                        ini(1) + 7, ini(2) + 0; ...
                        ini(1) + 5, ini(2) + 0];
    Map.Obstacles{4} = [ini(1) + 5, ini(2) + 5; ...
                        ini(1) + 5, ini(2) + 7; ...
                        ini(1) + 7, ini(2) + 7; ...
                        ini(1) + 7, ini(2) + 5; ...
                        ini(1) + 5, ini(2) + 5];
return;
end