Message = "Hello world";
MessageLength = strlength(Message) + 5;

msgSet = zeros(100 * MessageLength, 1);
for msgCnt = 0 : 99
    msgSet(msgCnt * MessageLength + (1 : MessageLength)) = ...
        sprintf('%s %03d\n', Message, msgCnt);
end
barkerSeq = [1;1;1;1;1;0;0;1;1;0;1;0;1];
barkerCode = [barkerSeq; barkerSeq];
%msgtest = int2bit(msgSet, 7);
MessageBits = [int2bit(msgSet, 7)];


% Receiver
rx = char(bit2int(MessageBits,7));
strings_2D = reshape(rx, 16, [])';
% Convert the 2D matrix into a cell array of strings
strings_cell = cellstr(strings_2D);

disp(strings_cell);
%res = char(int2str(rx))