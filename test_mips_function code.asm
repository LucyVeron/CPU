 
addi $t2, $t2, 2            # assign g       
addi $t3, $t3, 3           # assign h
       
 add $s0, $zero, $t2      # save original g
 add $s1, $zero, $t3      # save original h
  
 add $t2,$t2,$t2          # add g to itself to make 2*g

 add $t4,$t3,$t3          # add h to itself many times to make 5*h
 add $t5,$t4,$t3
 add $t6,$t5,$t3
 add $t7,$t6,$t3

 add $t8,$t2,$t7          # add 2*g and 5*h to make f
