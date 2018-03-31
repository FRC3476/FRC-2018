package org.usfirst.frc.team3476.utility;

public class RunningAverageQueue {
	public static void main(String[] args)
	{
		RunningAverageQueue q = new RunningAverageQueue(20);
		for (int i = 0; i < 20; i++)
			q.push(1);
		q.print();
		System.out.println(q.getAverage());
		for (int i = 0; i < 10; i++)
			q.push(3);
		q.print();
		System.out.println(q.getAverage());
		
		for (int i = 7; i < 14; i++)
			q.push(i * i);
		q.print();
		System.out.println(q.getAverage());
	}
	
	private int end;
	private final int size;
	private double[] data;
	private double total;
	
	public RunningAverageQueue(int size)
	{
		this.size = size;
		data = new double[size];
		end = 0;
		total = 0;
	}
	
	public void push(double num)
	{
		total += num;
		end = (end + 1) % size;
		total -= data[end];
		data[end] = num;
	}
	
	public double getAverage()
	{
		return total / size;
	}
	
	public void print()
	{
		for (int i = 0; i < size; i++)
			System.out.print(data[i] + " ");
		System.out.println();
	}
}
