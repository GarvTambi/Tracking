package template;

/* import table */
import logist.simulation.Vehicle;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.concurrent.atomic.AtomicInteger;

import javax.swing.JOptionPane;

import logist.agent.Agent;
import logist.behavior.DeliberativeBehavior;
import logist.plan.Plan;
import logist.task.Task;
import logist.task.TaskDistribution;
import logist.task.TaskSet;
import logist.topology.Topology;
import logist.topology.Topology.City;

/**
 * An optimal planner for one vehicle.
 */
@SuppressWarnings("unused")
public class DeliberativeTemplate implements DeliberativeBehavior {

	enum Algorithm { BFS, ASTAR }
	
	/* Environment */
	Topology topology;
	TaskDistribution td;
	ArrayList<State> finalstate_list_bfs= new ArrayList<State>();
	ArrayList<State> finalstate_list_astar= new ArrayList<State>();

	/* the properties of the agent */
	Agent agent;
	int capacity;

	/* the planning class */
	Algorithm algorithm;
	boolean debug = false;
	
	//Plan called utilities
	TaskSet tasksUnreachable = null;
	boolean planCancelled = false;
	@Override
	public void setup(Topology topology, TaskDistribution td, Agent agent) {
		this.topology = topology;
		this.td = td;
		this.agent = agent;
		
		// initialize the planner
		int capacity = agent.vehicles().get(0).capacity();
		this.capacity = capacity;
		String algorithmName = agent.readProperty("algorithm", String.class, "ASTAR");
		
		// Throws IllegalArgumentException if algorithm is unknown
		this.algorithm = Algorithm.valueOf(algorithmName.toUpperCase());
	}
	@Override
	public Plan plan(Vehicle vehicle, TaskSet tasks) {
		Plan plan = null;
		if(tasksUnreachable != null && planCancelled) tasks.removeAll(tasksUnreachable);
		else tasksUnreachable=tasks;
		// Compute the plan with the selected algorithm.
		switch (algorithm) {
		case ASTAR:
			int alpha = 5;
			try {
				plan = ASTARPlan(alpha, vehicle, tasks, tasksUnreachable,planCancelled);
			} catch (CloneNotSupportedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			break;
		case BFS:
			try {
				plan = BFSPlan(vehicle, tasks, tasksUnreachable,planCancelled);
			} catch (CloneNotSupportedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			break;
		default:
			throw new AssertionError("Should not happen.");
		}		
		return plan;
	}
	private Plan naivePlan(Vehicle vehicle, TaskSet tasks) {
		City current = vehicle.getCurrentCity();
		Plan plan = new Plan(current);

		for (Task task : tasks) {
			// move: current city => pickup location
			for (City city : current.pathTo(task.pickupCity))
				plan.appendMove(city);

			plan.appendPickup(task);

			// move: pickup location => delivery location
			for (City city : task.path())
				plan.appendMove(city);

			plan.appendDelivery(task);

			// set current city
			current = task.deliveryCity;
		}
		return plan;
	}

	private ArrayList<Action> FindBestAction(HashMap<ArrayList<Action>,Double> action_table) {
		ArrayList<Action> bestActionList = null;
		double minCost = Double.MAX_VALUE;

		for(Entry<ArrayList<Action>, Double> entry : action_table.entrySet()){
			if(entry.getValue() < minCost){
				minCost= entry.getValue();
				bestActionList= entry.getKey();
		    }
		}
		System.out.println("Min Cost= " + minCost);
		return bestActionList;
	}
	private ArrayList<Action> FindBestState(ArrayList<State> state_list) {
		ArrayList<Action> bestActionList = null;
		double minCost = Double.MAX_VALUE;

		for(int i=0;i<state_list.size();i++){
			if(state_list.get(i).getCost() < minCost){
				minCost = state_list.get(i).getCost();	
				bestActionList= state_list.get(i).getActionList();
		    }
		}
		//System.out.println("Min Cost= " + minCost);
		return bestActionList;
	}
	private Plan BuildingPlan(City initialCity, Plan plan, ArrayList<Action> action_list) {
		City currentCity = initialCity;
		for(int i=0;i<action_list.size();i++) {
			if(action_list.get(i).getCity()!=null) {
				currentCity = action_list.get(i).getCity();
				plan.appendMove(action_list.get(i).getCity());			
			}
			else {
				if(action_list.get(i).getPickup()) {
					plan.appendPickup(action_list.get(i).getTask());
				}
				else {					
					plan.appendDelivery(action_list.get(i).getTask());
				}
			}
		}
		return plan;
	}
	
	private void RemovingSimilarStates(ArrayList<State> state_list) {
		ArrayList<Integer> indexToDel = new ArrayList<Integer>();
		int i = 0;
		int j = 0;
		while(i<state_list.size()) {
			boolean i_removed=false;
			while(j<state_list.size()) {
				boolean j_removed=false;
				if(AreTaskTablesEqual(state_list.get(i).getTaskTable(),state_list.get(j).getTaskTable()) && i!=j && state_list.get(i).getCurrentCity() == state_list.get(j).getCurrentCity()) {
					if(state_list.get(i).getCost() > state_list.get(j).getCost()) {
						state_list.remove(i);
						i_removed = true;
						break;
					}
					else{
						state_list.remove(j);
						j_removed = false;
					}					
				}
				if(!j_removed) j++;
			}
			if(!i_removed) i++;
		}
	}
	
	private boolean AreTaskTablesEqual(Hashtable<Task,Boolean> task_table1, Hashtable<Task,Boolean> task_table2) {
		if(task_table1.size() == task_table2.size()) {
			for (Entry<Task, Boolean> entry1 : task_table1.entrySet()) {
				for (Entry<Task, Boolean> entry2 : task_table2.entrySet()) {
					if(entry1.getKey() == entry2.getKey() && entry1.getValue()!=entry2.getValue()) return false;
				}
			}
		}
		
		return true;
	}
	
	private ArrayList<State> RemovingStateWithHigherCost(ArrayList<State> state_list, double cost) {
		//System.out.println("RemovingStateWithHigherCost, init state_list_size: " + state_list.size());
		ArrayList<State> new_list = new ArrayList<State>();
		for(int i=0;i<state_list.size();i++) {
			if(state_list.get(i).getCost()<cost) {
				new_list.add(state_list.get(i));
			}
		}
		return new_list;
	}
	
	private int calculNaiveCost(Hashtable<Task,Boolean> task_table, Vehicle vehicle) {
		int cost = 0;
		City currentCity = vehicle.getCurrentCity();
		City finalCity = vehicle.getCurrentCity();
		for (Entry<Task, Boolean> entry : task_table.entrySet()) {
			// move: current city => pickup location
			for (City city : currentCity.pathTo(entry.getKey().pickupCity)) {
				cost+=(int)finalCity.distanceTo(city)*vehicle.costPerKm();
				finalCity = city;
			}
			currentCity = entry.getKey().pickupCity;
			// move: pickup location => delivery location
			for (City city : currentCity.pathTo(entry.getKey().deliveryCity))
			{
				cost+=(int)finalCity.distanceTo(city)*vehicle.costPerKm();
				finalCity = city;
			}

			// set current city
			currentCity = entry.getKey().deliveryCity;
		}
		return cost;
	}
	private Plan BFSPlan(Vehicle vehicle, TaskSet tasks, TaskSet tasksUnreachable, Boolean planCancelled) throws CloneNotSupportedException {			
		//Variables
		finalstate_list_bfs.clear();
		City currentCity = vehicle.getCurrentCity();
		ArrayList<Action> action_list = new ArrayList<Action>();
		ArrayList<State> state_list = new ArrayList<State>();
		Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
		State currentState = new State();		
		AtomicInteger currentSpace = new AtomicInteger(vehicle.capacity());
		int naiveCost = 0;
		//fetching all the tasks
		
		for (Task task : tasks) {
			task_table.put(task, true);
		}
		if(planCancelled) {
			for (Task task : tasksUnreachable) {
				task_table.put(task, false);
			}
		}
			
		int task_number = task_table.size();
		naiveCost = calculNaiveCost(task_table, vehicle);
		//Declaring initial state with initial city, vehicle space and all the tasks
		currentState = new State(currentCity, currentSpace.get(), task_table, action_list, 0);
		state_list.add(currentState);
		int state_number = 0;
		int count=0;
		//building the plan until there is no more task to pick up or to deliver
		while(!state_list.isEmpty()) {
			state_number=state_list.size();
			for(int i=0;i<state_number;i++) {	
				for (Entry<Task, Boolean> entry : state_list.get(i).task_table.entrySet()) {
					State newState = state_list.get(i).clone();
					City final_City = newState.getCurrentCity();
					int cost = 0;
					if(entry.getValue() == true && entry.getKey().weight < newState.getCurrentSpace()) {
						// move: current city => pickup location
						for (City city : newState.getCurrentCity().pathTo(entry.getKey().pickupCity)) {
							newState.action_list.add(new Action(false,false,null,true,city));
							newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
							final_City = city;
						}
						newState.action_list.add(new Action(true,false,entry.getKey(),true,null));
						newState.decreaseCurrentSpace(entry.getKey().weight);
						newState.setCurrentCity(final_City);
						newState.updatingTaskTable(entry.getKey(), false);
					}
					else if(entry.getValue() == false) {

						// move: pickup location => delivery location
						for (City city : newState.getCurrentCity().pathTo(entry.getKey().deliveryCity)) {
							newState.action_list.add(new Action(false,false,null,true,city));
							newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
							final_City = city;
						}
						newState.action_list.add(new Action(false,true,entry.getKey(),true,null));
						newState.setCurrentCity(final_City);
						newState.increaseCurrentSpace(entry.getKey().weight);
						newState.task_table.remove(entry.getKey());
					}
					else continue;
					if(newState.task_table.isEmpty()) {
						finalstate_list_bfs.add(newState);
					}
					else {
						state_list.add(newState);
					}						
				}	
			}
			for(int i=0;i<state_number;i++) {
				state_list.remove(0);
			}
			count++;
			double progress = (double)count*10 /(task_number*2);
			System.out.println("Progress: " + progress);
			RemovingSimilarStates(state_list);
			if(progress>3 && task_number>5) state_list = RemovingStateWithHigherCost(state_list,naiveCost*0.052*progress);
		}		
		//finding best action
		ArrayList<Action> bestActionList = FindBestState(finalstate_list_bfs);
		Plan plan = new Plan(vehicle.getCurrentCity());
		//building best plan with best action list
		plan=BuildingPlan(vehicle.getCurrentCity(), plan, bestActionList);		
		return plan;
	}
	private Plan ASTARPlan(int alpha, Vehicle vehicle, TaskSet tasks, TaskSet tasksUnreachable, Boolean planCancelled) throws CloneNotSupportedException {			
		//Variables
		finalstate_list_astar.clear();
		City currentCity = vehicle.getCurrentCity();
		ArrayList<Action> action_list = new ArrayList<Action>();
		ArrayList<State> state_list = new ArrayList<State>();
		ArrayList<State> transition_state_list = new ArrayList<State>();
		Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
		State currentState = new State();		
		AtomicInteger currentSpace = new AtomicInteger(vehicle.capacity());
		int naiveCost = 0;
		//fetching all the tasks
		for (Task task : tasks) {
			task_table.put(task, true);
		}
		if(planCancelled) {
			for (Task task : tasksUnreachable) {
				task_table.put(task, false);
			}
			alpha=1;
		}
		int task_number = task_table.size();
		naiveCost = calculNaiveCost(task_table, vehicle);
		//Declaring initial state with initial city, vehicle space and all the tasks
		currentState = new State(currentCity, currentSpace.get(), task_table, action_list, 0);
		state_list.add(currentState);
		int state_number = 0;
		int count=0;
		//building the plan until there is no more task to pick up or to deliver
		while(finalstate_list_astar.isEmpty()) {
			for(int j=0;j<alpha;j++) {
				state_number=state_list.size();
				for(int i=0;i<state_number;i++) {	
					for (Entry<Task, Boolean> entry : state_list.get(i).task_table.entrySet()) {
						State newState = state_list.get(i).clone();
						City final_City = newState.getCurrentCity();
						int cost = 0;
						if(entry.getValue() == true && entry.getKey().weight < newState.getCurrentSpace()) {
							// move: current city => pickup location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().pickupCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(true,false,entry.getKey(),true,null));
							newState.decreaseCurrentSpace(entry.getKey().weight);
							newState.setCurrentCity(final_City);
							newState.updatingTaskTable(entry.getKey(), false);
						}
						else if(entry.getValue() == false) {
	
							// move: pickup location => delivery location
							for (City city : newState.getCurrentCity().pathTo(entry.getKey().deliveryCity)) {
								newState.action_list.add(new Action(false,false,null,true,city));
								newState.increaseCost((int)final_City.distanceTo(city)*vehicle.costPerKm());
								final_City = city;
							}
							newState.action_list.add(new Action(false,true,entry.getKey(),true,null));
							newState.setCurrentCity(final_City);
							newState.increaseCurrentSpace(entry.getKey().weight);
							newState.task_table.remove(entry.getKey());
						}
						else continue;
						if(j==0) newState.setIdentity(count);
						else newState.setIdentity(state_list.get(i).getIdentity());
						if(newState.task_table.isEmpty()) {
							finalstate_list_astar.add(newState);
						}
						else {							
							state_list.add(newState);
						}
						count++;
					}
				}
				for(int i=0;i<state_number;i++) {
					transition_state_list.add(state_list.get(0));
					state_list.remove(0);
				}
			}
			if(!state_list.isEmpty() && !planCancelled) {
				State minimalCostState = new State();
				int indexStateMinCost = -1;
				int minCost=Integer.MAX_VALUE;
				for(int i=0;i<state_list.size();i++) {
					if(state_list.get(i).getCost()<minCost) {
						minCost=state_list.get(i).getCost();
						indexStateMinCost=state_list.get(i).getIdentity();
					}
				}
				for(int i=0;i<transition_state_list.size();i++) {
					if(transition_state_list.get(i).getIdentity()==indexStateMinCost) {
						minimalCostState=transition_state_list.get(i);
						break;
					}
				}
				state_list.clear();
				transition_state_list.clear();
				state_list.add(minimalCostState);
			}
		}		
		//finding best action

		ArrayList<Action> bestActionList = FindBestState(finalstate_list_astar);
		Plan plan = new Plan(vehicle.getCurrentCity());
		//building best plan with best action list
		plan=BuildingPlan(vehicle.getCurrentCity(), plan, bestActionList);	
		return plan;
	}
	
	@Override
	public void planCancelled(TaskSet carriedTasks) {
		System.out.println("********************");
		System.out.println("Plan CANCELLLED");

		if (!carriedTasks.isEmpty()) {
			// This cannot happen for this simple agent, but typically
			// you will need to consider the carriedTasks when the next
			// plan is computed.
			System.out.println("carriedTasks:" + carriedTasks.toString());
			this.tasksUnreachable.clear();
			this.tasksUnreachable.addAll(carriedTasks);	
			planCancelled=true;
		}
	}
}
class Action{
	boolean pickup=false;
	boolean delivery=false;
	Task task=null;
	boolean move=false;
	City city = null;
	public Action(boolean pickup,boolean delivery, Task task, boolean move, City city) {				
		this.pickup = pickup;
		this.delivery = delivery;
		this.task = task;
		this.move = move;		
		this.city = city;				
	}
	public Action() {					
	}
	public boolean getPickup() {
		return this.pickup;
	}
	public boolean getDelivery() {
		return this.delivery;
	}
	public boolean getMove() {
		return this.move;
	}
	public Task getTask() {
		return this.task;
	}
	public City getCity() {
		return this.city;
	}
}
class State implements Cloneable {
	private City currentCity;
	private int currentSpace;
	Hashtable<Task,Boolean> task_table = new Hashtable<Task,Boolean>();
	ArrayList<Action> action_list = new ArrayList<Action>();
	int cost = 0;
	int identity = -1;
	public State clone() throws CloneNotSupportedException {
		State clonedObj = (State) super.clone();
        clonedObj.currentCity = this.currentCity;
        clonedObj.currentSpace = new Integer(this.currentSpace);
        clonedObj.cost = new Integer(this.cost);
        clonedObj.task_table = new Hashtable<Task,Boolean>(this.task_table);
        clonedObj.action_list = new ArrayList<Action>(this.action_list);
        return clonedObj;
    }
	public State(City currentcity, int currentSpace, Hashtable<Task,Boolean> task_table, ArrayList<Action> action_list, int cost) {				
		this.currentCity = currentcity;
		this.currentSpace = currentSpace;
		this.task_table = task_table;		
		this.action_list = action_list;		
		this.cost = cost;				
	}
	public String toString() {
		return "\nIdentity: " + this.identity + " Current cost= " + this.getCost() +" Current space= " + this.getCurrentSpace() + "\n\t task table= " + this.getTaskTable().toString();

	}
	public State() {				
						
	}
	public City getCurrentCity() {
		return this.currentCity;
	}
	public void setCurrentCity(City city) {
		this.currentCity=city;
	}
	public int getIdentity() {
		return this.identity;
	}
	public void setIdentity(int identity) {
		this.identity=identity;
	}

	public int getCurrentSpace() {
		return this.currentSpace;
	}
	public int getCost() {
		return this.cost;
	}
	public void increaseCost(int cost) {
		this.cost += cost;
	}
	public void decreaseCost(int cost) {
		this.cost -= cost;
	}
	public void decreaseCurrentSpace(int space) {
		this.currentSpace -= space;
	}
	public void increaseCurrentSpace(int space) {
		this.currentSpace += space;
	}

	public Hashtable<Task,Boolean> getTaskTable() {
		return this.task_table;
	}
	public void updatingTaskTable(Task task, boolean bool) {
		for (Entry<Task, Boolean> entry : this.task_table.entrySet()) {
			if(entry.getKey() == task) entry.setValue(bool);
		}
	}
	public ArrayList<Action> getActionList() {
		return this.action_list;
	}
}
