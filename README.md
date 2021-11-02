
    Go to room 1, then go to room 2 or 3, finally go to room 4, avoid room 5 all the time
    Testing task:
        sub1: go to room 1 while avoiding room 5
        if success:
            sub2: go to room 2 while avoiding room 5
            if success:
                sub3: go to room 4 while avoiding room 5
                if success:
                    Verified
                else:
                    failed
            else:
                sub4: go to room 3 while avoiding room 5
                if success:
                    sub5: go to room 4 while avoiding room 5
                    if success:
                        Verified
                    else:
                        failed
                else:
                    failed
        else:
            failed            
