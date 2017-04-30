# RRT Algorithm

<pre>
RRT(<em>x<sub>init</sub></em>, <em>x<sub>goal</sub></em>, <em>map</em>):
  
  // Initialize the tree
  <em>tree</em> <- [<em>x<sub>init</sub></em>]
  <em>x<sub>init</sub></em>.<em>cost</em> <- 0
  
  // Grow the tree until it reaches the goal
  While:
    If steer(<em>tree</em>[0],<em>x<sub>goal</sub></em>) exists:
      <em>x<sub>goal</sub></em>.<em>parent</em> <- <em>tree</em>[0]
      Break
    Choose a random state <em>x<sub>rand</sub></em>
    Find the state <em>x</em> ϵ <em>tree</em> that minimizes:
      <em>x<sub>rand</sub></em>.<em>cost</em> = <em>x</em>.<em>cost</em> + |steer(<em>x</em>,<em>x<sub>rand</sub></em>)|
    If <em>x</em> exists:
      <em>tree</em> <- [<em>x<sub>rand</sub></em>] ∪ <em>tree</em> 
      <em>x<sub>rand</sub></em>.<em>parent</em> <- <em>x</em>
      
  // Back track to get the path to goal
  <em>path</em> <- [<em>x<sub>goal</sub></em>]
  while <em>path</em>[0].<em>parent</em> exists:
    <em>path</em> <- [<em>path</em>[0].<em>parent</em>] ∪ <em>path</em>
    
  // Greedily optimize the path
  Until the time limit is reached:
    Choose a random state <em>x<sub>rand</sub></em>
    For each set of states [<em>x</em><sub>-1</sub>,<em>x</em><sub>0</sub>,<em>x</em><sub>1</sub>] ⊆ <em>path</em>:
      If |steer(<em>x</em><sub>-1</sub>,<em>x</em><sub>0</sub>)| + |steer(<em>x</em><sub>0</sub>,<em>x</em><sub>1</sub>)| < |steer(<em>x</em><sub>-1</sub>,<em>x<sub>rand</sub></em>)| + |steer(<em>x<sub>rand</sub></em>,<em>x</em><sub>1</sub>)|:
        <em>x</em><sub>0</sub> <- <em>x<sub>rand</sub></em>
  
  return <em>path</em>
</pre>

<pre>
steer(<em>x</em>, <em>x<sub>goal</sub></em>, <em>map</em>):
  TODO:
  That thing with the circles
  Fast marching with car model at state.
</pre>
